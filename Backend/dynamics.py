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

def velocity_correction(precession_state, reaction_wheels, inertia, t_max, t_start, sample_frequency):
    final_state = precession_state

    RW_radius = reaction_wheels["radius"]
    RW_mass = reaction_wheels["mass"]
    RW_RPM = reaction_wheels["max_rpm"]
    RW_spin_up = reaction_wheels["max_spinup_rate"]
    RW_inertia = 0.5 * RW_mass * RW_radius**2
    RPM_current = [0,0,0]
    RPM_output = [[],[],[]]

    t_end = t_start + 0.05
    t_split = np.linspace(t_start, t_end, int(sample_frequency*(t_end-t_start)))
    t_output = []
    y_output = [[],[],[],[],[],[],[]]

    while ((abs(final_state[4]) > 10**(-3)) or (abs(final_state[5]) > 10**(-3)) or (abs(final_state[6]) > 10**(-3))) and (t_end <= t_max):
        w_x, w_y, w_z = final_state[4:]

        momentum_x = inertia[0] * w_x
        momentum_y = inertia[1] * w_y
        momentum_z = inertia[2] * w_z

        RPM_required_x = (momentum_x / RW_inertia) * (30/np.pi)
        RPM_required_y = (momentum_y / RW_inertia) * (30/np.pi)
        RPM_required_z = (momentum_z / RW_inertia) * (30/np.pi)

        if abs(RPM_required_x)/(t_end - t_start) > RW_spin_up:
            RPM_x = (RPM_required_x/abs(RPM_required_x)) * RW_spin_up * (t_end - t_start) + RPM_current[0]
        else:
            RPM_x = RPM_required_x + RPM_current[0]

        if abs(RPM_required_y)/(t_end - t_start) > RW_spin_up:
            RPM_y = (RPM_required_y/abs(RPM_required_y)) * RW_spin_up * (t_end - t_start) + RPM_current[1]
        else:
            RPM_y = RPM_required_y + RPM_current[1]

        if abs(RPM_required_z)/(t_end - t_start) > RW_spin_up:
            RPM_z = (RPM_required_z/abs(RPM_required_z)) * RW_spin_up * (t_end - t_start) + RPM_current[2]
        else:
            RPM_z = RPM_required_z + RPM_current[2]

        torque_x = - RW_inertia * np.pi/30 * (RPM_x - RPM_current[0]) / (t_end - t_start)
        torque_y = - RW_inertia * np.pi/30 * (RPM_y - RPM_current[1]) / (t_end - t_start)
        torque_z = - RW_inertia * np.pi/30 * (RPM_z - RPM_current[2]) / (t_end - t_start)

        solution = integrate(final_state, inertia, [torque_x, torque_y, torque_z], [t_start, t_end])
        y_values = solution.sol(t_split)

        t_output = np.concatenate([t_output, t_split])
        y_output = np.concatenate([y_output, y_values], axis=1)
        RPM_output[0] = np.concatenate([RPM_output[0], [RPM_x for i in t_split]])
        RPM_output[1] = np.concatenate([RPM_output[1], [RPM_y for i in t_split]])
        RPM_output[2] = np.concatenate([RPM_output[2], [RPM_z for i in t_split]])

        final_state = y_output[:, -1]
        RPM_current = [RPM_x, RPM_y, RPM_z]
        
        t_start = t_end
        t_end = t_end + 0.05
        t_split = np.linspace(t_start, t_end, int(sample_frequency*(t_end-t_start)))

    return [t_output, y_output, RPM_output]
