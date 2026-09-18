// ADCS Simulator frontend: Three.js scene, user inputs, backend calls.

// TODO: config value for backend URL, set once backend is deployed
const BACKEND_URL = "http://localhost:5000";

let scene, camera, renderer, satelliteMesh;

function initScene() {
    // TODO: set up Three.js scene, camera, renderer, drag controls
}

function createSatelliteMesh(dimensions) {
    // TODO: box geometry sized from user-supplied dimensions
}

function readUserInputs() {
    // TODO: read initial angular velocity, mass, dimensions, wheel params,
    // disturbance torque magnitude/direction from the form
}

async function runSimulation(params) {
    // TODO: POST params to `${BACKEND_URL}/simulate`, return the response JSON
}

function animateSatellite(stateTimeSeries) {
    // TODO: step the satellite mesh through returned orientation quaternions
}

document.getElementById("correct-attitude-btn")?.addEventListener("click", () => {
    // TODO: wire up correct-attitude flow
});

initScene();
