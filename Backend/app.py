"""Flask entry point and API routes for the ADCS simulator backend."""

from flask import Flask, request, jsonify
from flask_cors import CORS

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
    # TODO: run simulation
    return jsonify({"status": "not implemented"})


if __name__ == "__main__":
    app.run(debug=True)
