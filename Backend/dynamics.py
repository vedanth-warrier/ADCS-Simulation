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


def integrate(state0, inertia, external_torque, t_span, reltol = 1e-3, abstol = 1e-6):
    """Integrate the equations of motion over t_span using scipy.

    TODO: wire up solve_ivp with equations_of_motion.
    """

    return solve_ivp(equations_of_motion, t_span, state0, args=(inertia, external_torque), dense_output=True, atol = abstol, rtol = reltol)

def saturation_function(t, t_sat, RPM_max, external_torque):
    if external_torque != 0:
        RPM_list = [(external_torque/abs(external_torque)) * RPM_max * i / t_sat if i<t_sat else (external_torque/abs(external_torque)) *  RPM_max for i in t]
        sat_list = [False if i<t_sat else True for i in t]
    else:
        RPM_list = [0 for i in t]
        sat_list = [False for i in t]

    return [RPM_list, sat_list]


def correction(precession_state, reaction_wheels, inertia, t_max, t_start, sample_frequency, Kp, Kd):
    final_state = np.concatenate([precession_state[:4]/np.linalg.norm(precession_state[:4]), precession_state[4:]])

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

    saturated = [False, False, False]
    ran = False
    
    while (
        (abs(final_state[0]) > 10**(-3)) or 
        (abs(final_state[1]) > 10**(-3)) or 
        (abs(final_state[2]) > 10**(-3)) or 
        (abs(final_state[3]) > 1 + 10**(-3) or abs(final_state[3]) < 1 - 10**(-3)) or 
        (abs(final_state[4]) > 10**(-3)) or 
        (abs(final_state[5]) > 10**(-3)) or 
        (abs(final_state[6]) > 10**(-3))
        ) and (t_end <= t_max):

        ran = True

        orientation = np.array(final_state[:4])
        error = np.array([-orientation[0], -orientation[1], -orientation[2], orientation[3]])
        if error[3] < 0:
            error = -error
        
        w_body = final_state[4:]

        external_torque = Kp*error[:3] - Kd*w_body
        spin_rates_required = (external_torque / RW_inertia) * (30/np.pi)
        for i in range(3):
            if abs(spin_rates_required[i]) > RW_spin_up:
                external_torque[i] = spin_rates_required[i]/abs(spin_rates_required[i]) * RW_spin_up * RW_inertia * (np.pi/30)
        RPM_x = - (external_torque[0] * (t_end - t_start)) / (RW_inertia * np.pi/30) + RPM_current[0]
        if abs(RPM_x) >= RW_RPM:
            RPM_x = RPM_x/abs(RPM_x) * RW_RPM
            external_torque[0] = 0
        RPM_y = - (external_torque[1] * (t_end - t_start)) / (RW_inertia * np.pi/30) + RPM_current[1]
        if abs(RPM_y) >= RW_RPM:
            RPM_y = RPM_y/abs(RPM_y) * RW_RPM
            external_torque[1] = 0
        RPM_z = - (external_torque[2] * (t_end - t_start)) / (RW_inertia * np.pi/30) + RPM_current[2]
        if abs(RPM_z) >= RW_RPM:
            RPM_z = RPM_z/abs(RPM_z) * RW_RPM
            external_torque[2] = 0

        solution = integrate(final_state, inertia, external_torque, [t_start, t_end])
        y_values = solution.sol(t_split)

        t_output = np.concatenate([t_output, t_split])
        y_output = np.concatenate([y_output, y_values], axis=1)
        RPM_output[0] = np.concatenate([RPM_output[0], [RPM_x for i in t_split]])
        RPM_output[1] = np.concatenate([RPM_output[1], [RPM_y for i in t_split]])
        RPM_output[2] = np.concatenate([RPM_output[2], [RPM_z for i in t_split]])

        final_state = np.concatenate([y_output[:, -1][:4] / np.linalg.norm(y_output[:, -1][:4]) , y_output[:, -1][4:]])
        RPM_current = [RPM_x, RPM_y, RPM_z]
        
        t_start = t_end
        t_end = t_end + 0.05
        t_split = np.linspace(t_start, t_end, int(sample_frequency*(t_end-t_start)))

    for i in range(3):
        if ran == True and abs(RPM_output[i][-1]) == RW_RPM:
            saturated[i] = True

    return [t_output, y_output, RPM_output, saturated, ran]

def long_timeframe(t_length, unstable_t_length, torque, direction_vector, reaction_wheels, frequency):
    RW_radius = reaction_wheels["radius"]
    RW_mass = reaction_wheels["mass"]
    RW_RPM = reaction_wheels["max_rpm"]
    RW_inertia = 0.5 * RW_mass * RW_radius**2

    external_torque = direction_vector/np.linalg.norm(direction_vector) * torque

    t_sat = [0,0,0]
    for i in range(3):
        if external_torque[i] != 0:
            t_sat[i] = (RW_RPM * np.pi/30) * RW_inertia / abs(external_torque[i])

    time_scale = max(t_sat)/(t_length - unstable_t_length)
    t_output = np.linspace(0, t_length, int(frequency*(t_length)))
    RPM_output = [[0],[0],[0]]
    saturation_output = [[],[],[]]

    for i in range(3):
        sat_func = saturation_function(t_output*time_scale, t_sat[i], RW_RPM, external_torque[i])
        RPM_output[i] = sat_func[0]
        saturation_output[i] = sat_func[1]
    
    return [time_scale, t_output, RPM_output, saturation_output]