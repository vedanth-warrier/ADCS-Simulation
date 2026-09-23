"""Flask entry point and API routes for the ADCS simulator backend."""

from flask import Flask, request, jsonify
from flask_cors import CORS
import dynamics
import numpy as np

app = Flask(__name__)
CORS(app)


@app.route("/simulate", methods=["POST"])
def simulate():
    """Accept simulation inputs, run the physics, return the state time series.

    TODO: parse request.json for satellite/wheel parameters, initial
    conditions and timeframe, call into dynamics.py and controller.py, and
    return orientation quaternions plus wheel torque/spin data as JSON.
    """
    params = request.get_json()
    frequency = 60
    sample_frequency = 120
    t_precession_end = 5
    stability_end = 5
    correction_max_time = 500

    state_initial = np.array([
        params['current_orientation'][0], params['current_orientation'][1], params['current_orientation'][2], params['current_orientation'][3],
        params['initial_angular_velocity']['x'], params['initial_angular_velocity']['y'], params['initial_angular_velocity']['z']
            ])

    mass = params['satellite']['mass']

    dimensions = np.array([
        params['satellite']['dimensions']['x'], 
        params['satellite']['dimensions']['y'], 
        params['satellite']['dimensions']['z']
            ])

    reaction_wheel_params = params["reaction_wheels"]

    inertia = dynamics.moment_of_inertia_box(mass+3*reaction_wheel_params['mass'], dimensions)

    precession_solution = dynamics.integrate(state_initial, inertia, [0,0,0], [0, t_precession_end])
    t_precession = np.linspace(0, t_precession_end, t_precession_end*frequency)
    y_precession = precession_solution.sol(t_precession)
    RPM_precession = [[0 for i in t_precession] for j in range(3)]
    text_precession = ['Torque-Free Precession' for i in t_precession]

    state_post_precession = y_precession[:, -1]
    t_correction, y_correction, RPM_correction = dynamics.velocity_correction(state_post_precession, reaction_wheel_params, inertia, correction_max_time, t_precession_end, sample_frequency)
    text_correction = ['Applying Correction' for i in t_correction]

    state_post_correction = y_correction[:, -1]
    time_post_correction = t_correction[-1]
    stability_solution = dynamics.integrate(state_post_correction, inertia, [0,0,0], [time_post_correction, time_post_correction + stability_end])
    t_stability = np.linspace(time_post_correction, time_post_correction + stability_end, stability_end*frequency)
    y_stability = stability_solution.sol(t_stability)
    RPM_stability = [[RPM_correction[j][-1] for i in t_stability] for j in range(3)]
    text_stability = ['Stability Achieved' for i in t_stability]

    time = np.concatenate([t_precession, t_correction, t_stability]).tolist()
    y_values = np.concatenate([y_precession, y_correction, y_stability], axis=1).tolist()
    RPM_values = np.concatenate([RPM_precession, RPM_correction, RPM_stability], axis=1).tolist()
    text = np.concatenate([text_precession, text_correction, text_stability]).tolist()

    quarts = [list(row) for row in zip(y_values[0], y_values[1], y_values[2], y_values[3])]
    final_quarts = [(i/np.linalg.norm(i)).tolist() for i in quarts]
    
    return jsonify({
        "time": time,
        "orientation": final_quarts,
        "rpm": {
            "x": RPM_values[0],
            "y": RPM_values[1],
            "z": RPM_values[2],
        },
        "saturated": {
            "x": False,
            "y": False,
            "z": False
        },
        "text": text
            })


if __name__ == "__main__":
    # Not 5000: on macOS that port is usually held by the AirPlay Receiver
    # (ControlCenter), which silently swallows requests to localhost:5000
    # instead of a "port in use" error, since Flask's default host binds
    # only IPv4 (127.0.0.1) while AirPlay also holds the IPv6 loopback,
    # which is what "localhost" resolves to first.
    app.run(debug=True, port=5001)
