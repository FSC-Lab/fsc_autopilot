import functools

import geometric_controller
import jax
import jax.experimental.compilation_cache.compilation_cache as cc
import jax.numpy as jnp
import matplotlib.pyplot as plt
import numpy as np
from trajectory_generation import MinimumSnap

np.set_printoptions(4, suppress=True)

cc.initialize_cache("./.cache")

ctrl = geometric_controller.GeometricCtrl(
    max_acc=np.inf, k_pos=jnp.array([10, 10, 100])
)

ms = MinimumSnap(5, [0, 0, 1, 1])


@jax.jit
def quaternion_product(lhs, rhs):
    return jnp.array(
        [
            lhs[3] * rhs[0] + lhs[0] * rhs[3] + lhs[1] * rhs[2] - lhs[2] * rhs[1],
            lhs[3] * rhs[1] + lhs[1] * rhs[3] + lhs[2] * rhs[0] - lhs[0] * rhs[2],
            lhs[3] * rhs[2] + lhs[2] * rhs[3] + lhs[0] * rhs[1] - lhs[1] * rhs[0],
            lhs[3] * rhs[3] - lhs[0] * rhs[0] - lhs[1] * rhs[1] - lhs[2] * rhs[2],
        ]
    )


@functools.partial(jax.jit, static_argnums=[2])
def quaternion_rotate_point(quaternion, point, invert_rotation=False):
    vec = jax.lax.select(invert_rotation, -quaternion[0:3], quaternion[0:3])
    uv = jnp.cross(vec, point)
    uv += uv
    return point + quaternion[3] * uv + jnp.cross(vec, uv)


@functools.partial(
    jax.jit, static_argnames=("dynamics", "method", "return_derivatives")
)
def forward_dynamics(dynamics, x0, u, dt, method="euler", return_derivatives=False):
    """Run forward simulation of a dynamical system

    Details
    -------
    This function enables time-varying parameters (known beforehand) to be passed in,
    whereas constant parameters are intended to be put into the sys object

    Parameters
    ----------
    sys : ModelBase
        An object satisfying the ModelBase Interface
    x0 : ArrayLike
        Initial state
    u : ArrayLike
        A sys.nu-by-len(dt) array of control inputs
    dt : ArrayLike
        An array of time steps
    method: Literal["RK4"] | Literal["euler"]
        Specifies the integration method
    Returns
    -------
    Tuple[jnp.array, jnp.array] | jnp.array
        The state and optionally observation trajectory
    """

    def _update(x_op, tup):
        u, dt = tup
        dx = dynamics(x_op, u)
        if method == "RK4":
            k = jnp.empty((4, x_op.size))
            k = k.at[0, :].set(dx)
            k = k.at[1, :].set(dynamics(x_op + dt / 2 * k[0, :], u))
            k = k.at[2, :].set(dynamics(x_op + dt / 2 * k[1, :], u))
            k = k.at[3, :].set(dynamics(x_op + dt * k[2, :], u))
            increment = jnp.array([1, 2, 2, 1]) @ k / 6
        elif method == "euler":
            increment = dx
        else:
            raise NotImplementedError(f"{method} is not a valid integration method")
        x_new = x_op + dt * increment
        if return_derivatives:
            return x_new, (x_new, dx)
        return x_new, x_new

    if u.ndim == 1:
        _, x = _update(x0, (u, dt))
        return x

    _, x = jax.lax.scan(_update, init=x0, xs=(u, dt))
    return x


class Quadrotor:
    def __init__(self, mass):
        self._mass = mass

    @functools.partial(jax.jit, static_argnums=[0], donate_argnums=(1,))
    def dynamics(self, x, u):
        q = x[3:7]
        v = x[7:10]

        f = jnp.array([0.0, 0.0, u[0] / self._mass])
        w = jnp.array([u[1], u[2], u[3], 0.0]) / 2.0
        g = jnp.array([0.0, 0.0, -9.81])

        dx = jnp.empty(10)
        dx = dx.at[0:3].set(v)
        dx = dx.at[3:7].set(quaternion_product(q, w))
        dx = dx.at[7:10].set(quaternion_rotate_point(q, f) + g)
        return dx


quad = Quadrotor(1.0)

state = np.r_[np.zeros(3), np.zeros(3), 1.0, np.zeros(3)]
# state = np.zeros(6)
pos_ref = np.array(
    [
        [0, 0, 0],
        [0, 0, 10],
        [10, 0, 10],
        [10, 10, 10],
        [0, 10, 10],
        [0, 0, 10],
        [0, 0, 0],
    ]
)

t_ref = np.array([0, 5, 10, 15, 20, 25, 30]) * 5
traj = ms.generate(pos_ref.T, t_ref)
real_traj = traj.to_real_trajectory(1.0, np.r_[t_ref[0] : t_ref[-1], 0.1])

integrator = functools.partial(
    forward_dynamics, quad.dynamics, return_derivatives=False, method="RK4"
)


pos = []
for pos_ref, vel_ref in zip(real_traj.position.T, real_traj.velocity.T):
    vel = state[7:10]
    att = state[3:7]

    for _ in range(10):
        thrust_command, q_des, ang_vel_cmd = ctrl.command_loop(
            state[0:3], att, vel, pos_ref, vel_ref, np.zeros(3)
        )
        state = integrator(state, np.r_[thrust_command, ang_vel_cmd], 0.01)
    pos.append(state[0:3])

pos = np.array(pos)

fig, ax = plt.subplots(subplot_kw={"projection": "3d"})
ax.plot(*np.hsplit(pos, 3))
pos = real_traj.position.T
ax.plot(*np.hsplit(pos, 3), "--")
plt.show()
# print(att_cmd)
# print(state[0:3])
