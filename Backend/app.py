from flask import Flask, request, jsonify
from flask_cors import CORS
import dynamics
import numpy as np
import scipy.interpolate as interpolate
import os

app = Flask(__name__)
CORS(app)  # frontend and backend run on different origins, so this has to be open


# takes the simulation inputs from the frontend, runs the physics for whichever
# timeframe was selected, and returns the resulting time series as JSON
@app.route("/simulate", methods=["POST"])
def simulate():
    params = request.get_json()
    frequency = 60          # sample rate (Hz) for the time series sent back to the frontend
    sample_frequency = 120  # internal sub-step sampling rate used inside the correction loop

    if params['timeframe'] == 'seconds':
        t_precession_end = 5
        stability_end = 5
        correction_max_time = 100

        # state = [qx, qy, qz, qw, wx, wy, wz], matches dynamics.equations_of_motion
        state_initial = np.array([
            params['current_orientation'][0],
            params['current_orientation'][1],
            params['current_orientation'][2],
            params['current_orientation'][3],
            params['initial_angular_velocity']['x'],
            params['initial_angular_velocity']['y'],
            params['initial_angular_velocity']['z']
                ])

        mass = params['satellite']['mass']  # excludes the reaction wheels, added in below

        dimensions = np.array([
            params['satellite']['dimensions']['x'],
            params['satellite']['dimensions']['y'],
            params['satellite']['dimensions']['z']
                ])

        reaction_wheel_params = params["reaction_wheels"]

        # wheel mass counted x3 since all three wheels are mounted on the satellite body
        inertia = dynamics.moment_of_inertia_box(
            mass+3*reaction_wheel_params['mass'],
            dimensions
        )

        # phase 1: free tumble under the initial angular velocity, no correction torque yet
        precession_solution = dynamics.integrate(
            state_initial,
            inertia,
            [0,0,0],
            [0, t_precession_end],
            [0,0,0]
        )
        # sample the continuous solution onto a fixed grid the frontend can animate smoothly
        t_precession = np.linspace(0, t_precession_end, t_precession_end*frequency)
        y_precession = precession_solution.sol(t_precession)
        RPM_precession = [[0 for i in t_precession] for j in range(3)]  # wheels are idle here
        text_precession = ['Torque-Free Precession' for i in t_precession]

        state_post_precession = y_precession[:, -1]  # last sampled state feeds into phase 2

        # phase 2: reaction wheels correct the attitude back to zero rotation
        # the trailing 1, 1 are the PD gains (Kp, Kd), fixed rather than user-configurable
        t_correction, y_correction, RPM_correction, saturated, ran = dynamics.correction(
            state_post_precession,
            reaction_wheel_params,
            inertia,
            correction_max_time,
            t_precession_end,
            sample_frequency,
            1,
            1
        )
        text_correction = ['Applying Correction' for i in t_correction]

        # correction() can bail out having run nothing if it started already converged
        if ran:
            state_post_correction = y_correction[:, -1]
            time_post_correction = t_correction[-1]
            wheel_momentum_post_correction = 0.5 * reaction_wheel_params['mass'] * reaction_wheel_params["radius"]**2 * np.pi/30 * np.array(RPM_correction)[:, -1]
        else:
            state_post_correction = state_post_precession
            time_post_correction = t_precession_end
            wheel_momentum_post_correction = [0,0,0]

        # phase 3: hold the corrected attitude, to confirm it's actually settled
        stability_solution = dynamics.integrate(
            state_post_correction,
            inertia,
            [0,0,0],
            [time_post_correction, time_post_correction + stability_end],
            wheel_momentum_post_correction
            
        )
        t_stability = np.linspace(time_post_correction, time_post_correction + stability_end, stability_end*frequency)
        y_stability = stability_solution.sol(t_stability)

        # hold each wheel at whatever RPM it ended the correction phase on, since
        # there's no more torque being applied to change it during this hold phase
        if ran:
            RPM_stability = [[RPM_correction[j][-1] for i in t_stability] for j in range(3)]
        else:
            RPM_stability = [[0 for i in t_stability] for j in range(3)]

        # if any axis maxed out during correction, that wheel ran out of room before
        # fully cancelling the tumble, so the "stable" state isn't a clean zero-error one
        if np.any(saturated):
            text_stability = ['Saturated' for i in t_stability]
        else:
            text_stability = ['Stability Achieved' for i in t_stability]

        # stitch all three phases into one continuous time series
        time = np.concatenate([t_precession, t_correction, t_stability]).tolist()
        y_values = np.concatenate([y_precession, y_correction, y_stability], axis=1).tolist()
        RPM_values = np.concatenate([RPM_precession, RPM_correction, RPM_stability], axis=1).tolist()
        text = np.concatenate([text_precession, text_correction, text_stability]).tolist()

        # renormalise every quaternion before sending it out, guards against drift
        # picked up over the length of the run
        quarts = [list(row) for row in zip(y_values[0], y_values[1], y_values[2], y_values[3])]
        final_quarts = [(i/np.linalg.norm(i)).tolist() for i in quarts]

        # one JSON payload covering all three phases back to back, the frontend just
        # plays through "time" in order and doesn't need to know where one phase ends
        # and the next starts (that's what "text" is for, as a per-sample status label)
        return jsonify({
            "time": time,
            "orientation": final_quarts,
            "rpm": {
                "x": RPM_values[0],
                "y": RPM_values[1],
                "z": RPM_values[2],
            },
            "saturated": {
                "x": saturated[0],
                "y": saturated[1],
                "z": saturated[2]
            },
            "text": text
                })

    else:
        # adaptive mode: no orientation here, just each wheel's RPM ramp toward
        # saturation under a constant disturbance torque
        torque = params['disturbance_torque']['magnitude']
        direction_vector = list(params['disturbance_torque']['direction'].values())
        reaction_wheel_params = params["reaction_wheels"]

        # 100 and 5 are the playback window length and the "unstable" lead-in buffer,
        # both in playback seconds, see long_timeframe's time_scale comment for why
        time_scale, time, RPM, saturation = dynamics.long_timeframe(
            100,
            5,
            torque,
            direction_vector,
            reaction_wheel_params,
            frequency
        )

        return jsonify({
            "time": time.tolist(),
            "time_scale": time_scale,  # lets the frontend label the axis in real seconds/hours/days
            "rpm": {
                "x": RPM[0],
                "y": RPM[1],
                "z": RPM[2]
            },
            "saturated": {
                "x": saturation[0],
                "y": saturation[1],
                "z": saturation[2]
            }
        })


if __name__ == "__main__":
    # gunicorn (used in production) never runs this block, debug defaults off
    # so a live deploy doesn't accidentally expose Werkzeug's debugger
    app.run(debug=os.environ.get("FLASK_DEBUG") == "1", port=5001)
