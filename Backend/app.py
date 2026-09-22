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
        params['current_orientation'][0], params['current_orientation'][1], params['current_orientation'][2], params['current_orientation'][3],
        params['initial_angular_velocity']['x'], params['initial_angular_velocity']['y'], params['initial_angular_velocity']['z']
            ])

    t_span = 30

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

    y = dynamics.integrate(state_initial, inertia, [0,0,0], [0,t_span], 0)

    time = np.linspace(0, t_span, t_span*60)
    solution = y.sol(time)

    quarts = [list(row) for row in zip(solution[0], solution[1], solution[2], solution[3])]
    final_quarts = [(i/np.linalg.norm(i)).tolist() for i in quarts]

    text = ['Torque-Free Precession' for i in time]
    
    return jsonify({
        "time": time.tolist(),
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
