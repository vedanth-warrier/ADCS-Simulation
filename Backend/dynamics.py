import numpy as np
from scipy.integrate import solve_ivp


# diagonal moment of inertia tensor for a solid rectangular box, from total mass and side lengths (m)
# only the diagonal terms matter here since the box's axes are assumed to line up with the
# body axes, so there's no need for the full 3x3 inertia tensor with off-diagonal products
def moment_of_inertia_box(mass, dimensions):
    X,Y,Z = dimensions

    # standard box inertia formula, each axis depends on the other two side lengths
    I_xx = mass/12 * (Y**2 + Z**2)
    I_yy = mass/12 * (X**2 + Z**2)
    I_zz = mass/12 * (X**2 + Y**2)

    return [I_xx, I_yy, I_zz]


# state = [qx, qy, qz, qw, wx, wy, wz] (quaternion orientation + body angular velocity)
# returns d(state)/dt: quaternion kinematics plus Euler's rigid body equations
def equations_of_motion(t, state, inertia, external_torque, wheel_momentum):
    # quaternion derivative, standard dq/dt = 0.5 * omega(w) * q
    x_dot = 0.5*(state[3]*state[4] + state[1]*state[6] - state[2]*state[5])
    y_dot = 0.5*(state[3]*state[5] + state[2]*state[4] - state[0]*state[6])
    z_dot = 0.5*(state[3]*state[6] + state[0]*state[5] - state[1]*state[4])
    w_dot = -0.5*(state[0]*state[4] + state[1]*state[5] + state[2]*state[6])

    # Euler's rigid body equations, angular acceleration per axis with gyroscopic
    # coupling between axes (this is what produces precession on a non-principal spin)
    w_dot_x = (external_torque[0] - (inertia[2] - inertia[1])*state[5]*state[6] - (state[5]*wheel_momentum[2] - state[6]*wheel_momentum[1]))/inertia[0]
    w_dot_y = (external_torque[1] - (inertia[0] - inertia[2])*state[6]*state[4] - (state[6]*wheel_momentum[0] - state[4]*wheel_momentum[2]))/inertia[1]
    w_dot_z = (external_torque[2] - (inertia[1] - inertia[0])*state[4]*state[5] - (state[4]*wheel_momentum[1] - state[5]*wheel_momentum[0]))/inertia[2]

    return np.array([
        x_dot, y_dot, z_dot, w_dot,
        w_dot_x, w_dot_y, w_dot_z
            ])


# runs equations_of_motion over t_span with scipy. dense_output lets the caller sample
# the solution at any time afterwards via .sol(t) instead of only at fixed steps
# reltol/abstol are loose on purpose, tight tolerances make solve_ivp painfully slow
# for these long tumble/correction runs without actually changing the visible result
def integrate(state0, inertia, external_torque, t_span, wheel_momentum, reltol = 1e-3, abstol = 1e-6):
    return solve_ivp(
        equations_of_motion,
        t_span,
        state0,
        args=(inertia, external_torque, wheel_momentum),
        dense_output=True,
        atol = abstol,
        rtol = reltol
    )


# closed-form RPM ramp for one wheel under a constant torque: linear from 0 up to RPM_max,
# then flat once it hits t_sat (saturation time). Used instead of numerical integration
# since a constant torque makes the ramp analytically simple
def saturation_function(t, t_sat, RPM_max, external_torque):
    # (external_torque/abs(external_torque)) is just extracting the sign, so the wheel
    # ramps toward +RPM_max or -RPM_max depending on which way the torque pushes it
    if external_torque != 0:
        RPM_list = [
            (external_torque/abs(external_torque)) * RPM_max * i / t_sat if i<t_sat
            else (external_torque/abs(external_torque)) *  RPM_max
            for i in t
        ]
        # once t passes t_sat the wheel is maxed out, mark every point after that as saturated
        sat_list = [False if i<t_sat else True for i in t]
    else:
        # no torque on this axis at all, so the wheel just sits at 0 forever
        RPM_list = [0 for i in t]
        sat_list = [False for i in t]

    return [RPM_list, sat_list]


# PD attitude correction loop for "seconds" mode: drives the satellite from precession_state
# back to zero rotation (identity quaternion, zero angular velocity) using the reaction wheels,
# stepping forward in small time chunks until converged or t_max is hit
def correction(precession_state, reaction_wheels, inertia, t_max, t_start, sample_frequency, Kp, Kd):
    # renormalise the incoming quaternion in case of drift from the precession integration
    final_state = np.concatenate([
        precession_state[:4]/np.linalg.norm(precession_state[:4]),
        precession_state[4:]
    ])

    # same reaction wheel spec used for all three axes, since the sim assumes identical wheels
    RW_radius = reaction_wheels["radius"]
    RW_mass = reaction_wheels["mass"]
    RW_RPM = reaction_wheels["max_rpm"]
    RW_spin_up = reaction_wheels["max_spinup_rate"]
    RW_inertia = 0.5 * RW_mass * RW_radius**2  # solid disk moment of inertia

    RPM_current = [0,0,0]  # each wheel's current speed, carried across loop iterations
    RPM_output = [[],[],[]]  # full RPM history per wheel, built up chunk by chunk

    # advance in small fixed 0.05s chunks rather than one big integration, so the
    # controller can recompute torque and re-check saturation every chunk
    t_end = t_start + 0.05
    t_split = np.linspace(t_start, t_end, int(sample_frequency*(t_end-t_start)))
    t_output = []
    y_output = [[],[],[],[],[],[],[]]  # 7 rows to match the state vector layout

    saturated = [False, False, False]
    ran = False
    wheel_momentum = [0,0,0]

    # keep stepping while orientation/velocity are still meaningfully off zero, or time runs out
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

        # error quaternion against identity (zero rotation) is just the negated vector part,
        # flipped to the shortest-path sign so the controller doesn't fight itself
        orientation = np.array(final_state[:4])
        error = np.array([-orientation[0], -orientation[1], -orientation[2], orientation[3]])
        if error[3] < 0:
            error = -error

        w_body = final_state[4:]
        Kp_inertia = Kp*np.array(inertia)
        Kd_inertia = Kd*np.array(inertia)

        #Feedforward term:
        T_ff_x = (inertia[2]-inertia[1])*w_body[1]*w_body[2] + (w_body[1]*wheel_momentum[2] - w_body[2]*wheel_momentum[1])
        T_ff_y = (inertia[0]-inertia[2])*w_body[2]*w_body[0] + (w_body[2]*wheel_momentum[0] - w_body[0]*wheel_momentum[2])
        T_ff_z = (inertia[1]-inertia[0])*w_body[0]*w_body[1] + (w_body[0]*wheel_momentum[1] - w_body[1]*wheel_momentum[0])
        T_ff = np.array([T_ff_x, T_ff_y, T_ff_z])


        # PD control law: correction torque = Kp * orientation error - Kd * angular velocity
        external_torque = Kp_inertia*error[:3] - Kd_inertia*w_body + T_ff

        # convert the torque this would need into a wheel spin-up rate (rad/s -> RPM,
        # hence the 30/pi), so it can be compared against the wheel's physical limit
        spin_rates_required = (external_torque / RW_inertia) * (30/np.pi)

        # clip torque per axis if it would need a faster spin-up than the wheel can do
        for i in range(3):
            if abs(spin_rates_required[i]) > RW_spin_up:
                external_torque[i] = (
                    spin_rates_required[i]/abs(spin_rates_required[i])
                    * RW_spin_up * RW_inertia * (np.pi/30)
                )

        # step each wheel's RPM by this chunk's torque (negative sign because spinning the
        # wheel up one way pushes the body the other way), then clip to RW_RPM and zero the
        # torque out on that axis if it's already maxed out, since it has nothing left to give
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

        wheel_momentum = RW_inertia * np.pi/30 * np.array([RPM_x, RPM_y, RPM_z])

        # integrate this chunk with the (possibly clipped) correction torque applied
        solution = integrate(final_state, inertia, external_torque, [t_start, t_end], wheel_momentum)
        y_values = solution.sol(t_split)

        # append this chunk's samples onto the running output, wheel RPM is held flat
        # across the chunk since it's only updated once per chunk, not continuously
        t_output = np.concatenate([t_output, t_split])
        y_output = np.concatenate([y_output, y_values], axis=1)
        RPM_output[0] = np.concatenate([RPM_output[0], [RPM_x for i in t_split]])
        RPM_output[1] = np.concatenate([RPM_output[1], [RPM_y for i in t_split]])
        RPM_output[2] = np.concatenate([RPM_output[2], [RPM_z for i in t_split]])

        # renormalise again before the next chunk, carry the wheel speeds forward
        final_state = np.concatenate([
            y_output[:, -1][:4] / np.linalg.norm(y_output[:, -1][:4]) ,
            y_output[:, -1][4:]
        ])
        RPM_current = [RPM_x, RPM_y, RPM_z]

        t_start = t_end
        t_end = t_end + 0.05
        t_split = np.linspace(t_start, t_end, int(sample_frequency*(t_end-t_start)))

    # an axis is flagged saturated if it ended the run pinned at max RPM
    for i in range(3):
        if ran == True and abs(RPM_output[i][-1]) == RW_RPM:
            saturated[i] = True

    # ran is passed back out so the caller can tell "converged instantly, loop never
    # ran" apart from "actually corrected something"
    return [t_output, y_output, RPM_output, saturated, ran]


# "adaptive" mode: no orientation modelled here, just how fast each wheel's RPM ramps
# toward saturation under a constant disturbance torque, rescaled onto a fixed playback window
def long_timeframe(t_length, unstable_t_length, torque, direction_vector, reaction_wheels, frequency):
    RW_radius = reaction_wheels["radius"]
    RW_mass = reaction_wheels["mass"]
    RW_RPM = reaction_wheels["max_rpm"]
    RW_inertia = 0.5 * RW_mass * RW_radius**2  # solid disk moment of inertia

    # normalise the direction vector first so torque only sets the overall magnitude,
    # then split it across the three axes by the direction vector's components
    external_torque = direction_vector/np.linalg.norm(direction_vector) * torque

    # time for each axis's wheel to reach max RPM under its share of the torque
    # (angular momentum = torque * time), left at 0 for an axis with no torque on it
    t_sat = [0,0,0]
    for i in range(3):
        if external_torque[i] != 0:
            t_sat[i] = (RW_RPM * np.pi/30) * RW_inertia / abs(external_torque[i])

    # compress the real saturation time down into the playback window (t_length minus
    # a short unstable_t_length buffer at the start), driven by whichever axis takes
    # the longest to saturate. time_scale converts playback seconds back to real seconds
    time_scale = max(t_sat)/(t_length - unstable_t_length)

    # t_output is the actual playback timeline sent to the frontend, sampled at "frequency" Hz
    t_output = np.linspace(0, t_length, int(frequency*(t_length)))
    RPM_output = [[0],[0],[0]]
    saturation_output = [[],[],[]]

    # build each axis's RPM/saturation curve on the same compressed timeline, scaling
    # t_output back up to real seconds first so saturation_function sees true physical time
    for i in range(3):
        sat_func = saturation_function(t_output*time_scale, t_sat[i], RW_RPM, external_torque[i])
        RPM_output[i] = sat_func[0]
        saturation_output[i] = sat_func[1]

    return [time_scale, t_output, RPM_output, saturation_output]
