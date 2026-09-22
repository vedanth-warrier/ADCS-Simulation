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

    state_initial = np.array([
        params['current_orientation'][0], params['current_orientation'][1], params['current_orientation'][2],
        params['initial_angular_velocity']['x'], params['initial_angular_velocity']['y'], params['initial_angular_velocity']['z']
            ])

    mass = params['satellite']['mass']

    dimensions = np.array([
        params['satellite']['dimensions']['x'], 
        params['satellite']['dimensions']['y'], 
        params['satellite']['dimensions']['z']
            ])

    reaction_wheel_1 = params['reaction_wheels']['x']

    reaction_wheel_2 = params['reaction_wheels']['y']

    reaction_wheel_3 = params['reaction_wheels']['z']

    inertia = dynamics.moment_of_inertia_box(mass, dimensions)

    y = dynamics.integrate(state_initial, inertia, [0,0,0], [0,100], 0)

    roll = y.y[0]
    pitch = y.y[1]
    yaw = y.y[2]

    final_x = (np.sin(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) - np.cos(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)).tolist()
    final_y = (np.cos(roll/2)*np.sin(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.cos(pitch/2)*np.sin(yaw/2)).tolist()
    final_z = (np.cos(roll/2)*np.cos(pitch/2)*np.sin(yaw/2) - np.sin(roll/2)*np.sin(pitch/2)*np.cos(yaw/2)).tolist()
    final_w = (np.cos(roll/2)*np.cos(pitch/2)*np.cos(yaw/2) + np.sin(roll/2)*np.sin(pitch/2)*np.sin(yaw/2)).tolist()

    final_quarts = [list(row) for row in zip(final_x, final_y, final_z, final_w)]
    
    return jsonify({
        "time": y.t.tolist(),
        "orientation": final_quarts,
        "rpm": {
            "x": 'placeholder',
            "y": 'placeholder',
            "z": 'placeholder',
        },
        "saturated": {
            "x": False,
            "y": False,
            "z": False
        }
            })


if __name__ == "__main__":
    # Not 5000: on macOS that port is usually held by the AirPlay Receiver
    # (ControlCenter), which silently swallows requests to localhost:5000
    # instead of a "port in use" error, since Flask's default host binds
    # only IPv4 (127.0.0.1) while AirPlay also holds the IPv6 loopback,
    # which is what "localhost" resolves to first.
    app.run(debug=True, port=5001)
