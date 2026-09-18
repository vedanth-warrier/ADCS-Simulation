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

function clampToRange(input, value) {
    if (input.min !== "" && value < parseFloat(input.min)) value = parseFloat(input.min);
    if (input.max !== "" && value > parseFloat(input.max)) value = parseFloat(input.max);
    return value;
}

document.querySelectorAll(".stepper-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
        const input = document.getElementById(btn.dataset.target);
        const step = parseFloat(input.step) || 1;
        const direction = parseFloat(btn.dataset.step);
        const next = Math.round(((parseFloat(input.value) || 0) + direction * step) * 1e6) / 1e6;
        input.value = clampToRange(input, next);
    });
});

document.querySelectorAll('input[type="number"]').forEach((input) => {
    input.addEventListener("change", () => {
        if (input.value === "") return;
        input.value = clampToRange(input, parseFloat(input.value));
    });
});

const timeframeWarning = document.getElementById("timeframe-warning");
document.querySelectorAll('input[name="timeframe"]').forEach((radio) => {
    radio.addEventListener("change", (event) => {
        timeframeWarning.hidden = event.target.value === "seconds";
    });
});

initScene();
