import functools

import jax
import jax.numpy as jnp
import jax.numpy.linalg as la
from jax.scipy.spatial.transform import Rotation


def acc_to_quaternion(acc_sp, yaw):
    proj_xb_des = jnp.array([jnp.cos(yaw), jnp.sin(yaw), 0.0])
    zb_des = acc_sp / la.norm(acc_sp)
    yb_des = jnp.cross(zb_des, proj_xb_des)
    yb_des /= la.norm(yb_des)
    xb_des = jnp.cross(yb_des, zb_des)
    xb_des /= la.norm(xb_des)

    rotmat = jnp.column_stack([xb_des, yb_des, zb_des])
    return Rotation.from_matrix(rotmat).as_quat()


class NonlinearGeometricControl:
    def __init__(self, attctrl_tau):
        self._attctrl_tau = attctrl_tau

    def run(self, curr_att, ref_att, ref_acc):
        rotmat = Rotation.from_quat(curr_att).as_matrix()
        rotmat_d = Rotation.from_quat(ref_att).as_matrix()

        def vee(mat):
            return jnp.array([mat[2, 1], mat[0, 2], mat[1, 0]])

        error_att = 0.5 * vee(rotmat_d.T @ rotmat - rotmat.T @ rotmat_d)
        desired_rate = -(2.0 / self._attctrl_tau) * error_att
        zb = rotmat[:, 2]

        desired_thrust = ref_acc.dot(zb)

        return desired_thrust, desired_rate


def get_velocity_yaw(velocity):
    return jnp.arctan2(velocity[1], velocity[0])


class GeometricCtrl:
    GRAVITY = jnp.array([0.0, 0.0, 9.81])

    def __init__(
        self,
        velocity_yaw=False,
        min_acc=1e-5,
        max_acc=20.0,
        drag_d=jnp.zeros(3),
        attctrl_constant=0.1,
        k_pos=jnp.array([1.0, 1.0, 10.0]),
        k_vel=jnp.array([1.5, 1.5, 3.3]),
        max_tilt_angle=jnp.deg2rad(45),
    ) -> None:
        self._velocity_yaw = velocity_yaw
        self._min_acc = min_acc
        self._max_acc = max_acc

        self._drag_d = drag_d

        attctrl_tau = attctrl_constant

        self._k_pos = k_pos
        self._k_vel = k_vel
        self._max_tilt_ratio = jnp.tan(max_tilt_angle)

        self._controller = NonlinearGeometricControl(attctrl_tau)

    @functools.partial(jax.jit, static_argnums=[0])
    def command_loop(
        self, mav_pos, mav_att, mav_vel, pos_ref, vel_ref, acc_ref, mav_yaw=None
    ):
        desired_acc, mav_yaw = self.control_position(
            mav_pos, mav_vel, pos_ref, vel_ref, acc_ref, mav_yaw
        )
        return self.control_attitude(mav_att, mav_yaw, desired_acc)

    def control_position(
        self, mav_pos, mav_vel, pos_ref, vel_ref, acc_ref, mav_yaw=None
    ):
        # Position Controller

        if mav_yaw is None:
            mav_yaw = get_velocity_yaw(mav_vel)

        q_ref = acc_to_quaternion(acc_ref + self.GRAVITY, mav_yaw)
        r_ref = Rotation.from_quat(q_ref).as_matrix()
        pos_error = mav_pos - pos_ref
        vel_error = mav_vel - vel_ref

        # feedforward term for trajectory error
        a_fb = self._k_pos * pos_error + self._k_vel * vel_error

        # Rotor Drag compensation
        a_rd = r_ref @ jnp.diag(self._drag_d) @ r_ref.T @ vel_ref

        # Reference acceleration
        a_des = self.accel_setpoint_shaping(-a_fb - a_rd + self.GRAVITY + acc_ref)

        return a_des, mav_yaw

    def control_attitude(self, mav_att, mav_yaw, acc_ref):
        # Reference attitude
        q_des = acc_to_quaternion(acc_ref, mav_yaw)

        thrust_command, desired_rate = self._controller.run(mav_att, q_des, acc_ref)

        return thrust_command, q_des, desired_rate

    def accel_setpoint_shaping(self, acc_sp):
        def saturated():
            # Lift alone saturates actuators, deliver as much lift as possible and
            # no lateral acc
            return jnp.array([0.0, 0.0, self._max_acc])

        def unsaturated():
            z_sp = jnp.maximum(acc_sp[2], self._min_acc)
            # Lift does not saturate actuators, aim to deliver requested lift
            # exactly while scaling back lateral acc
            max_lateral_acc = jnp.sqrt(self._max_acc**2 - z_sp**2)

            lateral_acc_at_max_tilt = abs(z_sp) * self._max_tilt_ratio
            max_lateral_acc = jnp.minimum(max_lateral_acc, lateral_acc_at_max_tilt)

            xy_sp = acc_sp[0:2]
            lateral_acc_sqnorm = xy_sp @ xy_sp

            xy_sp = jax.lax.select(
                lateral_acc_sqnorm > max_lateral_acc**2,
                xy_sp * max_lateral_acc / jnp.sqrt(lateral_acc_sqnorm),
                xy_sp,
            )
            return jnp.array([xy_sp[0], xy_sp[1], z_sp])

        return jax.lax.cond(acc_sp[2] < self._max_acc, unsaturated, saturated)
