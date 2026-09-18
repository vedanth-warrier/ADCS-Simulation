"""Rigid body dynamics: moment of inertia, equations of motion, integration.

Physics reference: box satellite (not sphere), arbitrary-axis torque-free
precession is the required test case, see CLAUDE.md.
"""

import numpy as np
from scipy.integrate import solve_ivp


def moment_of_inertia_box(mass, dimensions):
    """Return the 3x3 inertia tensor for a rectangular prism satellite.

    TODO: derive from mass and (length, width, height).
    """
    pass


def equations_of_motion(t, state, inertia, external_torque):
    """Return the time derivative of the body state (angular velocity, orientation).

    TODO: Euler's rigid body equations plus quaternion kinematics.
    """
    pass


def integrate(state0, inertia, external_torque, t_span, t_eval):
    """Integrate the equations of motion over t_span using scipy.

    TODO: wire up solve_ivp with equations_of_motion.
    """
    pass
