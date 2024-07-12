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
"""

import enum
import warnings
from typing import Tuple, Union

import numpy as np
from numpy.typing import ArrayLike
from scipy import optimize

from trajectories import NormalizedTime, PiecewisePolynomialTrajectory


class MinimumSnapAlgorithm(enum.Enum):
    CONSTRAINED = 0
    CLOSED_FORM = 1


class MinimumSnap:
    def __init__(
        self, degree, deriv_wts, algorithm=MinimumSnapAlgorithm.CLOSED_FORM
    ) -> None:
        self._degree = degree
        self._n_cfs = degree + 1
        self._n_poly = -1
        self._n_vars = -1
        self._dim = -1

        deriv_wts = np.asarray(deriv_wts, dtype=np.float64).squeeze()
        if deriv_wts.ndim > 1 or (deriv_wts < 0.0).any():
            raise ValueError(
                "Weights on derivatives must be a 1D list of nonnegative numbers"
            )
        self._deriv_wts = deriv_wts
        self._iszero_tol = 1e-8
        self._r_cts = 3

        self._algorithm = algorithm

    def _process_refs(self, pos_ref, vel_ref, acc_ref):
        pos_ref = np.asarray(pos_ref, dtype=np.float64)
        dim, n_points = pos_ref.shape
        if self._dim < 0:
            self._dim = pos_ref.shape[0]
        elif self._dim != dim:
            raise ValueError("Mismatch in dimensions between points")

        if n_points < 2:
            raise ValueError("Too few waypoints")

        return np.dstack(
            [
                pos_ref,
                self._process_higher_order_refs(pos_ref.shape, vel_ref),
                self._process_higher_order_refs(pos_ref.shape, acc_ref),
            ],
        )

    def _process_higher_order_refs(self, shape, ref_tup):
        refs = np.zeros(shape, dtype=np.float64)
        if ref_tup is None:
            refs[:, 1:-1] = np.nan
        else:
            val, ids = [np.asarray(it, dtype=np.float64) for it in ref_tup]
            ids = ids.squeeze()
            if ids.ndim != 1:
                raise ValueError("Index of references must be 1D")

            n_refs = ids.size
            if n_refs != val.shape[1]:
                raise ValueError("Mismatch between number of references and indices")

            if n_refs > shape[1]:
                raise ValueError("Too many references")

            if np.any(ids >= val.shape[1]):
                raise ValueError(
                    "Index of references exceeded number of position references"
                )
            refs[:, ids] = val
        return refs

    def generate(self, pos_ref, t_ref, vel_refs=None, acc_refs=None):
        refs = self._process_refs(pos_ref, vel_refs, acc_refs)

        t_ref = np.asarray(t_ref, dtype=np.float64).squeeze()

        self._n_poly = refs.shape[1] - 1
        t_ref = NormalizedTime(t_ref)

        self._n_vars = self._n_poly * self._n_cfs

        n_derivs = self._deriv_wts.size
        if n_derivs > self._degree:
            raise ValueError(
                "More derivatives than order of the polynomial are requested"
            )
        Q_all = np.zeros((n_derivs, self._n_vars, self._n_vars))
        for r, c_r in enumerate(self._deriv_wts):
            if c_r < self._iszero_tol:
                continue
            for i in range(self._n_poly):
                it, sent = i * self._n_cfs, (i + 1) * self._n_cfs
                Q_all[r, it:sent, it:sent] = self.compute_Q(r, t_ref.durations[i])

        Q_all = np.sum(self._deriv_wts[:, None, None] * Q_all, axis=0).squeeze()

        return (
            self._solve_constr(refs, t_ref, Q_all)
            if self._algorithm is MinimumSnapAlgorithm.CONSTRAINED
            else self._solve_unconstr(refs, t_ref, Q_all)
        )

    def _solve_unconstr(self, refs, t_ref, Q_all):
        polys = np.zeros((self._dim, self._n_cfs, self._n_poly))
        for d in range(self._dim):
            # compute Tk   Tk(i,j) = ts(i)^(j-1)

            # compute A (self._cont_order*2*self._n_poly) * (self._n_cfs*self._n_poly)
            # 1:p  2:pv  3:pva  4:pvaj  5:pvajs
            A = np.zeros((self._r_cts * 2 * self._n_poly, self._n_cfs * self._n_poly))
            for i in range(self._n_poly):
                it, sent = self._n_cfs * i, self._n_cfs * (i + 1)
                for r in range(self._r_cts):
                    A[self._r_cts * 2 * i + r, it:sent] = (
                        self.compute_tvec(r, 0) / t_ref.durations[i] ** r
                    )
                    A[self._r_cts * (2 * i + 1) + r, it:sent] = (
                        self.compute_tvec(r, 1) / t_ref.durations[i] ** r
                    )

            # compute M
            M = np.zeros(
                (self._n_poly * 2 * self._r_cts, self._r_cts * (self._n_poly + 1))
            )
            for i in range(self._n_poly):
                it, sent = 2 * self._r_cts * i, 2 * self._r_cts * (i + 1)
                it2, sent2 = self._r_cts * i, self._r_cts * (i + 2)
                M[it:sent, it2:sent2] = np.eye(2 * self._r_cts)

            # compute C
            num_d = self._r_cts * (self._n_poly + 1)
            C = np.eye(num_d)
            df = np.concatenate([refs[d, :, 0], refs[d, 0, 1:], refs[d, -1, 1:]])
            # fix all pos(self._n_poly+1) + start va(2) +  va(2)
            fix_idx = np.concatenate(
                [np.arange(0, num_d, 3), np.array([1, 2, num_d - 2, num_d - 1])]
            )
            free_idx = np.setdiff1d(np.arange(num_d), fix_idx)
            C = np.hstack([C[:, fix_idx], C[:, free_idx]])

            AiMC = np.linalg.solve(A, M @ C)
            R = AiMC.T @ Q_all @ AiMC

            n_fix = fix_idx.size
            # Rff = R[:n_fix, :n_fix]
            Rpp = R[n_fix:, n_fix:]
            Rfp = R[:n_fix, n_fix:]
            # Rpf = R[n_fix:, :n_fix]

            dp = -np.linalg.solve(Rpp, Rfp.T @ df)

            p = np.reshape(
                AiMC @ np.concatenate([df, dp]), (self._n_cfs, self._n_poly), order="F"
            )
            polys[d, :, :] = (1.0 / t_ref.durations[None]) ** np.arange(0, self._n_cfs)[
                ..., None
            ] * p

        return PiecewisePolynomialTrajectory(t_ref, polys)

    def _solve_constr(self, refs, t_ref, Q_all):
        polys = np.zeros((self._dim, self._n_cfs, self._n_poly))
        for d in range(self._dim):
            Aeq_0, beq_0 = self._compute_dynamical_constraints(refs[d, :], t_ref)
            Aeq_1, beq_1 = self._compute_continuity_constraints(t_ref)

            Aeq = np.vstack([Aeq_0, Aeq_1])
            beq = np.concatenate([beq_0, beq_1])

            constr = optimize.LinearConstraint(Aeq, beq, beq)  # type: ignore
            soln = optimize.minimize(
                lambda x: (x @ Q_all @ x) / 2,
                np.zeros(self._n_vars),
                constraints=constr,
                method="trust-constr",
                jac=lambda x: Q_all @ x,
                hess=lambda _: Q_all,
            )
            P = np.reshape(soln.x, (self._n_cfs, self._n_poly), order="F")
            polys[d, :, :] = (1.0 / t_ref.durations[None]) ** np.arange(0, self._n_cfs)[
                ..., None
            ] * P
        return PiecewisePolynomialTrajectory(t_ref, polys)

    def _compute_continuity_constraints(self, t_ref):
        Aeqs = np.zeros(((self._n_poly - 1) * 3, self._n_vars))
        beqs = np.zeros((self._n_poly - 1) * 3)
        for i in range(self._n_poly - 1):
            it, sent = self._n_cfs * i, self._n_cfs * (i + 2)
            for r in range(self._r_cts):
                tvec_l = self.compute_tvec(r, 1) / t_ref.durations[i] ** r
                tvec_r = self.compute_tvec(r, 0) / t_ref.durations[i + 1] ** r
                Aeqs[3 * i + r, it:sent] = np.concatenate([tvec_l, -tvec_r])

        return Aeqs, beqs

    def _compute_dynamical_constraints(self, refs, t_ref):
        n_constrain_orders = np.count_nonzero(~np.isnan(refs), axis=1)
        Aeq = np.zeros((n_constrain_orders.sum(), self._n_vars))
        beq = np.zeros(n_constrain_orders.sum())

        row_its = np.concatenate([np.zeros(1), n_constrain_orders.cumsum()])
        for i in range(self._n_poly + 1):
            idx, tau = t_ref.find_piece(t_ref[i])
            it, sent = self._n_cfs * idx, self._n_cfs * (1 + idx)
            for r in range(n_constrain_orders[i]):
                Aeq[row_its[i] + r, it:sent] = (
                    self.compute_tvec(r, tau) / t_ref.durations[idx] ** r
                )
                beq[row_its[i] + r] = refs[i, r]
        return Aeq, beq

    def compute_Q(self, r, tau):
        Q = np.zeros((self._n_cfs, self._n_cfs))

        i = np.arange(r, self._n_cfs, dtype=np.int32)[None, :]
        l = i.T
        m_seq = np.arange(0, r)[None, None, :]
        k = -2 * r + 1
        Q[i, l] = (
            np.prod((i[..., None] - m_seq) * (l[..., None] - m_seq), axis=-1)
            * tau**k
            / (k + i + l)
        )
        return Q

    def compute_tvec(self, r, t):
        tvec = np.zeros(self._n_cfs)
        n_seq = np.arange(r, self._n_cfs, dtype=np.int64)
        r_seq = np.arange(0, r, dtype=np.int64)
        tvec[n_seq] = np.prod(n_seq[None, :] - r_seq[:, None], axis=0) * t ** (
            n_seq - r
        )
        return tvec

    def generate_trajectories(self, t_refs, p_refs, t_samples):
        traj = []
        for p_ref, t_ref, t_sample in zip(p_refs, t_refs, t_samples):
            pp = self.generate(p_ref, t_ref)
            real_traj = pp.to_real_trajectory(1.0, t_sample)
            traj.append((real_traj.states, real_traj.inputs))
        states, inputs = map(list, zip(*traj))
        return np.stack(states), np.stack(inputs)
