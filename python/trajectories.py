"""
Copyright © 2023 FSC Lab

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.


Part of this work is derived from "data_driven_mpc"
https://github.com/uzh-rpg/data_driven_mpc
Licensed under the following terms

 Trajectory generation functions. For the circle, lemniscate and random trajectories.

This program is free software: you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation, either version 3 of the License, or (at your option) any later
version.
This program is distributed in the hope that it will be useful, but WITHOUT
ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
You should have received a copy of the GNU General Public License along with
this program. If not, see <http://www.gnu.org/licenses/>.
"""

import copy
import warnings

import numpy as np


class NormalizedTime:
    """
    Represents normalized time for optimization purposes
    """

    def __init__(self, time):
        self._time = np.asarray(time, np.float64)
        self._durations = np.diff(self._time)

    def __len__(self):
        return self._time.size

    def __array__(self):
        return self._time

    @property
    def durations(self):
        return self._durations

    def find_piece(self, query_point):
        if not self._time_in_range(query_point):
            warnings.warn("Query point is outside of bounds. Clamping")
            query_point = np.clip(query_point, self[0], self[-1])
        idx = np.flatnonzero(query_point >= self._time[:-1])[-1]
        nrm_time = (query_point - self._time[idx]) / self._durations[idx]
        return idx, nrm_time

    def __getitem__(self, query_point):
        return self._time[query_point]

    def _time_in_range(self, time):
        return self._time[0] <= time <= self._time[-1]


class Piece:
    """
    Represents a piece in a piecewise-polynomial trajectory
    """

    def __init__(self, coeffs):
        self._coeffs = np.asarray(coeffs, dtype=np.float64)
        self._degree = self._coeffs.shape[1] - 1

        # index sequence for raising to power and polynomial derivative coefficients

    def __repr__(self):
        return f"""Order: {self._degree}
Coefficients: \n{self._coeffs}"""

    @property
    def dim(self):
        return 3

    @property
    def degree(self):
        return self._degree

    @property
    def coeffs(self):
        return self._coeffs

    def get_position(self, time):
        return self.get(time, 0)

    def get_velocity(self, time):
        return self.get(time, 1)

    def get_acceleration(self, time):
        return self.get(time, 2)

    def get(self, time, r):
        if r == 0:
            return self._coeffs @ (time ** np.arange(0, self._degree + 1))
        n_seq = np.arange(r, self._degree + 1, dtype=np.int64)
        r_seq = np.arange(0, r, dtype=np.int64)
        return (
            np.prod(n_seq[None, :] - r_seq[:, None], axis=0)
            * self._coeffs[:, n_seq]
            @ time ** (n_seq - r)
        )

    def normalized_pos_coeffs(self, duration):
        return self._coeffs * duration ** np.r_[self._degree : -1 : -1]

    def normalized_vel_coeffs(self, duration):
        return self._coeffs[:, :-1] * (
            duration ** np.r_[self._degree : -1 : -1][:-1]
            * np.r_[self._degree : -1 : -1][:-1]
        )

    def normalized_acc_coeffs(self, duration):
        return self._coeffs[:, :-2] * (
            duration ** np.r_[self._degree : -1 : -1][:-2]
            * np.r_[self._degree : -1 : -1][:-2]
            * np.r_[self._degree : -1 : -1][1:-1]
        )


class PiecewisePolynomialTrajectory:
    """
    Represents a piecewise polynomial trajectory
    """

    def __init__(self, t_ref: NormalizedTime, coeffs):
        self._t_ref = t_ref

        self._n_pieces = len(self._t_ref) - 1
        coeffs = np.asarray(coeffs, dtype=np.float64)
        if coeffs.ndim == 2:
            coeffs = coeffs[None, :, :]
        self._dim, self._degree, n_pieces = coeffs.shape
        if n_pieces != self._n_pieces:
            raise ValueError("Mismatch between durations vector and coefficients")
        self._pieces = [Piece(coeffs[:, :, idx]) for idx in range(self._n_pieces)]

    def __len__(self):
        return self._n_pieces

    def __getitem__(self, idx):
        return self._pieces[idx]

    def to_real_trajectory(self, vehicle_mass, t_ref, yaw_derivatives=None):
        if self._dim != 3:
            raise ValueError("This method is only for 3D trajectories")

        t_ref = np.asarray(t_ref, dtype=np.float64)
        len_traj = t_ref.size
        traj_derivatives = np.zeros((4, self._dim, len_traj), dtype=np.float64)
        for k, t in enumerate(t_ref):
            idx, _ = self._t_ref.find_piece(t)

            for i in range(4):
                traj_derivatives[i, :, k] = self._pieces[idx].get(
                    t - self._t_ref[idx], i
                )

        traj_ref = np.zeros((10, len_traj))
        traj_ref[0:3, :] = np.squeeze(traj_derivatives[0, :, :])
        traj_ref[7:10, :] = np.squeeze(traj_derivatives[1, :, :])
        if yaw_derivatives is None:
            yaw_derivatives = np.zeros((2, len_traj), dtype=np.float64)

        traj_ref[3:7, :], u_ref = forward(
            traj_derivatives[1:, :, :],
            yaw_derivatives,
            vehicle_mass,
        )

        return MultirotorTrajectory(traj_ref, u_ref, t_ref)


class Trajectory:
    """
    Generic trajectory for robotic vehicles with reference states, inputs and time
    """

    def __init__(self, states, inputs, time):
        self.time = np.asarray(time, dtype=np.float64)
        self.states = np.asarray(states, dtype=np.float64)
        self.inputs = np.asarray(inputs, dtype=np.float64)

    def __len__(self):
        return self.time.size

    def __iadd__(self, other):
        if not self.states.size:
            self.states = self.states.reshape(other.states.shape[0], -1)

        if not self.inputs.size:
            self.inputs = self.inputs.reshape(other.inputs.shape[0], -1)
        self.states = np.concatenate([self.states, other.states], axis=1)
        self.inputs = np.concatenate([self.inputs, other.inputs], axis=1)
        self.time = np.concatenate([self.time, other.time])
        return self

    def __add__(self, other):
        res = copy.copy(self)
        res += other
        return res

    @property
    def time_interval(self):
        return np.diff(self.time)

    def get_reference_chunk(self, idx, n_nodes, reference_over_sampling=1):
        # Dense references
        ref_traj_chunk = self.states[
            :, idx : idx + (n_nodes + 1) * reference_over_sampling
        ]
        ref_u_chunk = self.inputs[:, idx : idx + n_nodes * reference_over_sampling]

        # Indices for down-sampling the reference to number of MPC nodes
        downsample_ref_ind = np.arange(
            0,
            min(reference_over_sampling * (n_nodes + 1), ref_traj_chunk.shape[1]),
            reference_over_sampling,
            dtype=int,
        )

        # Sparser references (same dt as node separation)
        ref_traj_chunk = ref_traj_chunk[:, downsample_ref_ind]
        ref_u_chunk = ref_u_chunk[
            :, downsample_ref_ind[: max(len(downsample_ref_ind) - 1, 1)]
        ]

        return ref_traj_chunk, ref_u_chunk


class MultirotorTrajectory(Trajectory):
    """
    A specialized trajectory for multirotors, adding convenient property accessors
    """

    @property
    def position(self):
        return self.states[0:3, :]

    @property
    def attitude(self):
        return self.states[3:7, :]

    @property
    def velocity(self):
        return self.states[7:10, :]

    @property
    def thrust(self):
        return self.inputs[0, :]

    @property
    def angular_velocity(self):
        return self.inputs[1:3, :]


def forward(traj_refs, yaw_refs, vehicle_mass, grav=9.81):
    grav_vector = np.array([[0.0], [0.0], [grav]])
    traj_refs = np.asarray(traj_refs, dtype=np.float64)
    traj_refs = np.atleast_3d(traj_refs)

    n_ders, n_dims, len_traj = traj_refs.shape
    if (n_ders, n_dims) != (3, 3):
        raise ValueError(
            "Trajectory references must be 3D kinematical derivatives stackedcolumnwise"
        )

    vel = np.atleast_2d(traj_refs[0, ...])
    acc = np.atleast_2d(traj_refs[1, ...])
    jer = np.atleast_2d(traj_refs[2, ...])

    psi = yaw_refs[0, ...]
    dpsi = yaw_refs[1, ...]

    inputs = np.empty((4, len_traj), dtype=np.float64)

    z = acc + grav_vector
    z_nrm = np.linalg.norm(z, axis=0, keepdims=True)
    z /= z_nrm

    dz = -np.cross(z, np.cross(z, jer, axis=0), axis=0) / z_nrm
    inputs[0, :] = np.sum(z * (vehicle_mass * (acc + grav_vector)), axis=0)

    tilt_den = np.sqrt(2.0 * (1.0 + z[2, :]))
    tilt = np.array([0.5 * tilt_den, -z[1, :] / tilt_den, z[0, :] / tilt_den])
    c_half_psi = np.cos(0.5 * psi)
    s_half_psi = np.sin(0.5 * psi)
    attitude = np.array(
        [
            tilt[1] * c_half_psi + tilt[2] * s_half_psi,
            tilt[2] * c_half_psi - tilt[1] * s_half_psi,
            tilt[0] * s_half_psi,
            tilt[0] * c_half_psi,
        ]
    )
    c_psi = np.cos(psi)
    s_psi = np.sin(psi)
    omg_den = z[2, :] + 1.0
    omg_term = dz[2, :] / omg_den
    inputs[1, :] = (
        dz[0, :] * s_psi
        - dz[1, :] * c_psi
        - (z[0, :] * s_psi - z[1, :] * c_psi) * omg_term
    )
    inputs[2, :] = (
        dz[0, :] * c_psi
        + dz[1, :] * s_psi
        - (z[0, :] * c_psi + z[1, :] * s_psi) * omg_term
    )
    inputs[3, :] = (z[1, :] * dz[0, :] - z[0, :] * dz[1, :]) / omg_den + dpsi

    return np.squeeze(attitude), np.squeeze(inputs)
