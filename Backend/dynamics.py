"""Rigid body dynamics: moment of inertia, equations of motion, integration.

Physics reference: box satellite (not sphere), arbitrary-axis torque-free
precession is the required test case, see CLAUDE.md.
"""

import numpy as np
from scipy.integrate import solve_ivp


def moment_of_inertia_box(mass, dimensions):
    """Return the inertia tensor of the box.
    """

    X,Y,Z = dimensions
    I_xx = mass/12 * (Y**2 + Z**2)
    I_yy = mass/12 * (X**2 + Z**2)
    I_zz = mass/12 * (X**2 + Y**2)
    return [I_xx, I_yy, I_zz]


def equations_of_motion(t, state, inertia, external_torque):
    """Return the time derivative of the body state (orientation, angular velocity).

    TODO: Euler's rigid body equations plus quaternion kinematics.
    """

    x_dot = 0.5*(state[3]*state[4] + state[1]*state[6] - state[2]*state[5])
    y_dot = 0.5*(state[3]*state[5] + state[2]*state[4] - state[0]*state[6])
    z_dot = 0.5*(state[3]*state[6] + state[0]*state[5] - state[1]*state[4])
    w_dot = -0.5*(state[0]*state[4] + state[1]*state[5] + state[2]*state[6])

    w_dot_x = (external_torque[0] - (inertia[2] - inertia[1])*state[5]*state[6])/inertia[0]
    w_dot_y = (external_torque[1] - (inertia[0] - inertia[2])*state[6]*state[4])/inertia[1]
    w_dot_z = (external_torque[2] - (inertia[1] - inertia[0])*state[4]*state[5])/inertia[2]

    return np.array([
        x_dot, y_dot, z_dot, w_dot,
        w_dot_x, w_dot_y, w_dot_z
            ])


def integrate(state0, inertia, external_torque, t_span):
    """Integrate the equations of motion over t_span using scipy.

    TODO: wire up solve_ivp with equations_of_motion.
    """

    return solve_ivp(equations_of_motion, t_span, state0, args=(inertia, external_torque), dense_output=True)
