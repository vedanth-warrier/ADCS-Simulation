// ADCS Simulator frontend: Three.js scene, user inputs, backend calls.

// TODO: update once the backend is deployed. Port 5001, not Flask's
// default 5000: on macOS 5000 is usually held by the AirPlay Receiver
// (ControlCenter), which "localhost" resolves to over IPv6 ahead of
// Flask's IPv4-only default bind, so requests silently never reach Flask.
const BACKEND_URL = "http://localhost:5001";

let scene, camera, renderer, controls, satelliteMesh;
let gizmoScene, gizmoCamera, gizmoRenderer;
let wheelSpeedCharts = {};
const clock = new THREE.Clock();

// "seconds" is the only timeframe with an initial tumble (see CLAUDE.md:
// hours/days start from a stationary attitude), kept in sync by the
// timeframe radio listener further down instead of re-querying the DOM
// every animation frame.
let currentTimeframe = "seconds";

// Paused while a Correct Attitude request is in flight, so the live
// preview doesn't keep spinning underneath a correction that's supposedly
// already happening.
let liveTumbleEnabled = true;

// Set by animateSatellite() while a returned trajectory is being played back,
// cleared once playback finishes. null means no playback in progress.
let playback = null;

const AXIS_COLORS = { x: "#ff6b6b", y: "#6bff8f", z: "#6ba8ff" };

function getSatelliteDimensions() {
    return {
        x: parseFloat(document.getElementById("sat-dim-x").value) || 1,
        y: parseFloat(document.getElementById("sat-dim-y").value) || 1,
        z: parseFloat(document.getElementById("sat-dim-z").value) || 1,
    };
}

function initScene() {
    const container = document.getElementById("scene-container");
    const width = container.clientWidth;
    const height = container.clientHeight;

    scene = new THREE.Scene();

    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);
    camera.position.set(2.2, 1.6, 2.8);
    camera.lookAt(0, 0, 0);

    renderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    renderer.setPixelRatio(window.devicePixelRatio || 1);
    renderer.setSize(width, height);
    renderer.outputEncoding = THREE.sRGBEncoding;
    container.appendChild(renderer.domElement);

    scene.add(new THREE.AmbientLight(0xffffff, 0.5));
    const keyLight = new THREE.DirectionalLight(0xffffff, 0.9);
    keyLight.position.set(3, 4, 5);
    scene.add(keyLight);
    const rimLight = new THREE.DirectionalLight(0x38bdf8, 0.4);
    rimLight.position.set(-4, -2, -3);
    scene.add(rimLight);

    satelliteMesh = createSatelliteMesh();
    satelliteMesh.scale.set(getSatelliteDimensions().x, getSatelliteDimensions().y, getSatelliteDimensions().z);
    scene.add(satelliteMesh);

    controls = configureTrackballControls(new THREE.TrackballControls(camera, renderer.domElement));

    initRollControls(container);
    initGizmo();

    new ResizeObserver(() => {
        const { clientWidth, clientHeight } = container;
        if (clientWidth === 0 || clientHeight === 0) return;
        camera.aspect = clientWidth / clientHeight;
        camera.updateProjectionMatrix();
        renderer.setSize(clientWidth, clientHeight);
        controls.handleResize();
    }).observe(container);

    animate();
}

function updateLiveTumble(dt) {
    if (!liveTumbleEnabled || currentTimeframe !== "seconds") return;

    const omega = new THREE.Vector3(fieldValue("omega-x"), fieldValue("omega-y"), fieldValue("omega-z"));
    const angle = omega.length() * dt;
    if (angle === 0) return;

    // Kinematic only: rotates at the current omega as a fixed body-frame
    // axis/rate each frame. It does not solve the actual torque-free
    // rigid-body equations, so it won't show real precession/wobble for
    // an asymmetric body, that only appears once the backend's simulation
    // is wired up and played back. This is just a live visual indicator
    // so the "Correct Attitude" button has a visible tumble to correct.
    const increment = new THREE.Quaternion().setFromAxisAngle(omega.normalize(), angle);
    satelliteMesh.quaternion.multiply(increment);
}

function animate() {
    requestAnimationFrame(animate);
    const dt = clock.getDelta();
    if (playback) {
        updatePlayback();
    } else {
        updateLiveTumble(dt);
    }
    controls.update();
    renderer.render(scene, camera);
    updateGizmo();
}

// The skin sheet is a 3x2 grid of equal cells (see Images/Satellite-Skin.jpeg):
// [control panel] [solar array]  [sensor dome]
// [hatch/vent]     [wiring/tanks][thrusters/antenna]
// Mapped onto the box's faces in THREE's BoxGeometry group order below.
// This assignment is arbitrary, swap the {col, row} pairs to put a
// different panel on a different face.
const SKIN_GRID = { cols: 3, rows: 2 };
const SKIN_FACES = [
    { axis: "+X", col: 0, row: 0 }, // right  -> control panel
    { axis: "-X", col: 1, row: 0 }, // left   -> solar array
    { axis: "+Y", col: 2, row: 0 }, // top    -> sensor dome
    { axis: "-Y", col: 0, row: 1 }, // bottom -> hatch/vent
    { axis: "+Z", col: 1, row: 1 }, // front  -> wiring/tanks
    { axis: "-Z", col: 2, row: 1 }, // back   -> thrusters/antenna
];

function applySkinTexture(mesh) {
    // TextureLoader.load() returns immediately, before the image has
    // actually finished loading, so building clones and swapping in the
    // per-face materials happens in the onLoad callback, once there is
    // real pixel data to clone and upload rather than an empty placeholder.
    new THREE.TextureLoader().load("../Images/Satellite-Skin.jpeg", (baseTexture) => {
        baseTexture.encoding = THREE.sRGBEncoding;

        mesh.material = SKIN_FACES.map(({ col, row }) => {
            const texture = baseTexture.clone();
            texture.needsUpdate = true;
            texture.encoding = THREE.sRGBEncoding;
            texture.wrapS = THREE.ClampToEdgeWrapping;
            texture.wrapT = THREE.ClampToEdgeWrapping;
            texture.repeat.set(1 / SKIN_GRID.cols, 1 / SKIN_GRID.rows);
            // Texture V=0 is the bottom of the source image, so a "top row"
            // cell (row 0 in the file) sits in the upper half of V-space.
            texture.offset.set(col / SKIN_GRID.cols, row === 0 ? 0.5 : 0);
            return new THREE.MeshStandardMaterial({ map: texture, metalness: 0.15, roughness: 0.7 });
        });
    });
}

function createSatelliteMesh() {
    // Unit cube, scaled per axis to match satellite dimensions. Keeps the
    // skin texture and body axes (child below) in sync automatically.
    const geometry = new THREE.BoxGeometry(1, 1, 1);
    const placeholderMaterial = new THREE.MeshStandardMaterial({ color: 0x94a3b8, metalness: 0.4, roughness: 0.6 });
    const mesh = new THREE.Mesh(geometry, placeholderMaterial);
    mesh.add(new THREE.AxesHelper(0.9));
    applySkinTexture(mesh);

    return mesh;
}

function configureTrackballControls(instance) {
    // TrackballControls rotates freely about whatever axis the drag
    // implies (derived fresh from the camera's current position/up on
    // every call), unlike OrbitControls which locks rotation to a fixed
    // world "up" pole. That's what makes sequential drags (e.g. up/down
    // then left/right) compose the way a hand-held object would, instead
    // of the second drag always spinning around the same fixed axis
    // regardless of the first.
    instance.dynamicDampingFactor = 0.08;
    instance.minDistance = 1;
    instance.maxDistance = 20;
    return instance;
}

function initRollControls(container) {
    // TrackballControls has no dedicated roll gesture, so shift+drag is
    // handled separately here: it rotates camera.up around the
    // camera-to-target axis, which tilts the rendered view (via the
    // lookAt() TrackballControls already does each frame) without
    // touching camera.position, so it doesn't fight the rotate logic.
    // Unlike OrbitControls, TrackballControls reads camera.up fresh on
    // every drag rather than caching a basis at construction, so a roll
    // doesn't leave it in a stale state afterwards.
    let rolling = false;
    let startAngle = 0;
    const upStart = new THREE.Vector3();

    function angleFromCenter(clientX, clientY) {
        const rect = container.getBoundingClientRect();
        const cx = rect.left + rect.width / 2;
        const cy = rect.top + rect.height / 2;
        return Math.atan2(clientY - cy, clientX - cx);
    }

    function onRollMove(event) {
        if (!rolling) return;
        const deltaAngle = angleFromCenter(event.clientX, event.clientY) - startAngle;
        const forward = new THREE.Vector3().subVectors(camera.position, controls.target).normalize();
        camera.up.copy(upStart).applyAxisAngle(forward, deltaAngle);
    }

    function onRollUp() {
        rolling = false;
        window.removeEventListener("pointermove", onRollMove);
        window.removeEventListener("pointerup", onRollUp);
    }

    // Capture-phase listener on the container (an ancestor of the canvas),
    // for both event families TrackballControls might be listening for on
    // the canvas itself, so a shift-held drag can be stopped before it
    // ever reaches TrackballControls and starts a rotate instead.
    function interceptForRoll(event) {
        if (!event.shiftKey) return;
        event.stopPropagation();
        event.preventDefault();
        rolling = true;
        startAngle = angleFromCenter(event.clientX, event.clientY);
        upStart.copy(camera.up);
        window.addEventListener("pointermove", onRollMove);
        window.addEventListener("pointerup", onRollUp);
    }

    container.addEventListener("pointerdown", interceptForRoll, true);
    container.addEventListener("mousedown", interceptForRoll, true);
}

function updateSatelliteScale() {
    if (!satelliteMesh) return;
    const dimensions = getSatelliteDimensions();
    satelliteMesh.scale.set(dimensions.x, dimensions.y, dimensions.z);
}

function makeAxisLabel(text, color) {
    const canvas = document.createElement("canvas");
    canvas.width = 64;
    canvas.height = 64;
    const ctx = canvas.getContext("2d");
    ctx.fillStyle = color;
    ctx.font = "bold 44px sans-serif";
    ctx.textAlign = "center";
    ctx.textBaseline = "middle";
    ctx.fillText(text, 32, 34);

    const sprite = new THREE.Sprite(new THREE.SpriteMaterial({
        map: new THREE.CanvasTexture(canvas),
        transparent: true,
        depthTest: false,
    }));
    sprite.scale.set(0.45, 0.45, 0.45);
    return sprite;
}

function initGizmo() {
    const gizmoEl = document.getElementById("axis-gizmo");
    const size = gizmoEl.clientWidth || 84;

    gizmoScene = new THREE.Scene();
    gizmoCamera = new THREE.PerspectiveCamera(70, 1, 0.1, 10);

    gizmoRenderer = new THREE.WebGLRenderer({ antialias: true, alpha: true });
    gizmoRenderer.setPixelRatio(window.devicePixelRatio || 1);
    gizmoRenderer.setSize(size, size);
    gizmoEl.appendChild(gizmoRenderer.domElement);

    gizmoScene.add(new THREE.AxesHelper(1));

    [
        { text: "X", color: AXIS_COLORS.x, position: [1.4, 0, 0] },
        { text: "Y", color: AXIS_COLORS.y, position: [0, 1.4, 0] },
        { text: "Z", color: AXIS_COLORS.z, position: [0, 0, 1.4] },
    ].forEach(({ text, color, position }) => {
        const label = makeAxisLabel(text, color);
        label.position.set(...position);
        label.scale.set(0.4, 0.4, 0.4);
        gizmoScene.add(label);
    });
}

function updateGizmo() {
    if (!gizmoCamera) return;
    gizmoCamera.position.copy(camera.position).sub(controls.target).normalize().multiplyScalar(3);
    gizmoCamera.up.copy(camera.up);
    gizmoCamera.lookAt(0, 0, 0);
    gizmoRenderer.render(gizmoScene, gizmoCamera);
}

function createWheelSpeedChart(canvasId, color) {
    return new Chart(document.getElementById(canvasId), {
        type: "line",
        data: {
            labels: [],
            datasets: [{
                data: [],
                borderColor: color,
                backgroundColor: color,
                borderWidth: 1.5,
                pointRadius: 0,
                tension: 0.15,
            }],
        },
        options: {
            responsive: true,
            maintainAspectRatio: false,
            animation: false,
            plugins: {
                legend: { display: false },
            },
            scales: {
                x: {
                    title: { display: true, text: "Time (s)", color: "#8b95a5", font: { size: 10 } },
                    ticks: { color: "#8b95a5", font: { size: 9 }, maxTicksLimit: 6 },
                    grid: { color: "rgba(139, 149, 165, 0.12)" },
                },
                y: {
                    // No min/max set, so Chart.js autoscales to whatever
                    // data updateWheelSpeedCharts() is given.
                    title: { display: true, text: "Speed (RPM)", color: "#8b95a5", font: { size: 10 } },
                    ticks: { color: "#8b95a5", font: { size: 9 } },
                    grid: { color: "rgba(139, 149, 165, 0.12)" },
                },
            },
        },
    });
}

function initWheelSpeedCharts() {
    wheelSpeedCharts = {
        x: createWheelSpeedChart("graph-wheel-x", AXIS_COLORS.x),
        y: createWheelSpeedChart("graph-wheel-y", AXIS_COLORS.y),
        z: createWheelSpeedChart("graph-wheel-z", AXIS_COLORS.z),
    };
}

// Expected shape once the backend /simulate endpoint is wired up:
// { time: [t0, t1, ...], rpm: { x: [...], y: [...], z: [...] } },
// all arrays the same length. Axes autoscale to whatever range is passed.
function updateWheelSpeedCharts(simulationData) {
    const { time, rpm } = simulationData;
    ["x", "y", "z"].forEach((axis) => {
        const chart = wheelSpeedCharts[axis];
        chart.data.labels = time;
        chart.data.datasets[0].data = rpm[axis];
        chart.update();
    });
}

function fieldValue(id) {
    return parseFloat(document.getElementById(id).value) || 0;
}

function readWheelInputs(axis) {
    return {
        mass: fieldValue(`wheel-${axis}-mass`),
        radius: fieldValue(`wheel-${axis}-radius`),
        max_rpm: fieldValue(`wheel-${axis}-max-rpm`),
    };
}

function readUserInputs() {
    const dimensions = getSatelliteDimensions();
    return {
        initial_angular_velocity: {
            x: fieldValue("omega-x"),
            y: fieldValue("omega-y"),
            z: fieldValue("omega-z"),
        },
        satellite: {
            mass: fieldValue("sat-mass"),
            dimensions,
        },
        reaction_wheels: {
            x: readWheelInputs("x"),
            y: readWheelInputs("y"),
            z: readWheelInputs("z"),
        },
        disturbance_torque: {
            magnitude: fieldValue("disturbance-magnitude"),
            direction: {
                x: fieldValue("disturbance-dir-x"),
                y: fieldValue("disturbance-dir-y"),
                z: fieldValue("disturbance-dir-z"),
            },
        },
        timeframe: document.querySelector('input[name="timeframe"]:checked').value,
        // The live preview (see updateLiveTumble) has likely rotated the
        // satellite away from identity by the time this button is clicked,
        // so the backend needs the attitude it's actually correcting from,
        // not just the original angular velocity, or the returned
        // trajectory won't line up with what's on screen when playback
        // starts.
        current_orientation: satelliteMesh.quaternion.toArray(),
    };
}

async function runSimulation(params) {
    const response = await fetch(`${BACKEND_URL}/simulate`, {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify(params),
    });
    if (!response.ok) {
        throw new Error(`Backend responded with ${response.status} ${response.statusText}`);
    }
    return response.json();
}

// Finds the returned sample index whose interval [times[i], times[i+1]]
// contains simTime. solve_ivp's adaptive stepping means the `time` array
// isn't evenly spaced, so this can't just index by a fixed frame rate, it
// has to search for the bracketing pair each call.
function sampleIndexAtSimTime(simTime) {
    const { times } = playback;
    if (simTime <= times[0]) return 0;
    if (simTime >= times[times.length - 1]) return times.length - 1;

    let i = 0;
    while (i < times.length - 1 && times[i + 1] < simTime) i++;
    return i;
}

// Slerps between the two returned orientation samples either side of simTime.
function quaternionAtSimTime(simTime) {
    const { times, quaternions } = playback;
    const i = sampleIndexAtSimTime(simTime);
    if (i >= times.length - 1) return quaternions[quaternions.length - 1];

    const span = times[i + 1] - times[i];
    const t = span === 0 ? 0 : (simTime - times[i]) / span;
    // slerpQuaternions() exists in this three.js build but silently returns
    // undefined instead of interpolating, so clone()+slerp() is used instead
    // (the older, more reliably supported instance method).
    return quaternions[i].clone().slerp(quaternions[i + 1], t);
}

// Maps the exact status strings the backend sends in stateTimeSeries.text to
// a display colour: red while precessing freely, yellow while the wheels are
// actively correcting, green once stable.
const STATUS_COLORS = {
    "Torque-Free Precession": "#ff6b6b",
    "Applying Correction": "#f5c518",
    "Stability Achieved": "#4ade80",
};

function updateStatusLabel(text) {
    if (!text) return;
    sceneStatusLabel.textContent = text;
    sceneStatusLabel.style.color = STATUS_COLORS[text] || "";
    sceneStatusLabel.hidden = false;
}

function updatePlayback() {
    const elapsedWall = (performance.now() - playback.wallStart) / 1000;
    // Played back 1:1 against simulated seconds (the returned time span
    // itself, not a fixed guess), so playback length tracks however long the
    // correction actually took rather than compressing/stretching it.
    const progress = playback.durationSeconds === 0 ? 1 : Math.min(elapsedWall / playback.durationSeconds, 1);
    const simTime = playback.simStart + progress * (playback.simEnd - playback.simStart);

    satelliteMesh.quaternion.copy(quaternionAtSimTime(simTime));
    if (playback.texts) updateStatusLabel(playback.texts[sampleIndexAtSimTime(simTime)]);

    if (progress >= 1) {
        // Correction has finished: hold this attitude rather than resuming
        // the live tumble preview, which would spin the satellite again
        // using the pre-correction angular velocity and misrepresent it as
        // still tumbling.
        playback = null;
    }
}

function animateSatellite(stateTimeSeries) {
    const { time, orientation, text } = stateTimeSeries;
    if (!time || !orientation || time.length === 0) {
        // Nothing to play back, don't leave the satellite frozen forever.
        liveTumbleEnabled = true;
        return;
    }

    const simStart = time[0];
    const simEnd = time[time.length - 1];

    playback = {
        times: time,
        quaternions: orientation.map(([x, y, z, w]) => new THREE.Quaternion(x, y, z, w)),
        texts: text,
        simStart,
        simEnd,
        durationSeconds: simEnd - simStart,
        wallStart: performance.now(),
    };
}

const correctAttitudeBtn = document.getElementById("correct-attitude-btn");
const resetBtn = document.getElementById("reset-btn");
const simulationStatus = document.getElementById("simulation-status");
const sceneStatusLabel = document.getElementById("scene-status");

resetBtn?.addEventListener("click", () => location.reload());

correctAttitudeBtn?.addEventListener("click", async () => {
    const params = readUserInputs();

    correctAttitudeBtn.disabled = true;
    liveTumbleEnabled = false;
    simulationStatus.hidden = true;
    simulationStatus.classList.remove("status-error");

    try {
        const result = await runSimulation(params);
        updateWheelSpeedCharts(result);
        animateSatellite(result);
    } catch (error) {
        // Nothing was actually corrected, so let the live preview carry on
        // from wherever it already was rather than leaving the satellite
        // frozen because of a failed request.
        liveTumbleEnabled = true;
        simulationStatus.textContent = `Could not reach the backend: ${error.message}`;
        simulationStatus.classList.add("status-error");
        simulationStatus.hidden = false;
    } finally {
        correctAttitudeBtn.disabled = false;
    }
});

function clampToRange(input, value) {
    if (input.min !== "" && value < parseFloat(input.min)) value = parseFloat(input.min);
    if (input.max !== "" && value > parseFloat(input.max)) value = parseFloat(input.max);
    return value;
}

const satelliteDimensionIds = ["sat-dim-x", "sat-dim-y", "sat-dim-z"];

document.querySelectorAll(".stepper-btn").forEach((btn) => {
    btn.addEventListener("click", () => {
        const input = document.getElementById(btn.dataset.target);
        const step = parseFloat(input.step) || 1;
        const direction = parseFloat(btn.dataset.step);
        const next = Math.round(((parseFloat(input.value) || 0) + direction * step) * 1e6) / 1e6;
        input.value = clampToRange(input, next);
        if (satelliteDimensionIds.includes(input.id)) updateSatelliteScale();
    });
});

document.querySelectorAll('input[type="number"]').forEach((input) => {
    input.addEventListener("change", () => {
        if (input.value === "") return;
        input.value = clampToRange(input, parseFloat(input.value));
        if (satelliteDimensionIds.includes(input.id)) updateSatelliteScale();
    });
});

const timeframeWarning = document.getElementById("timeframe-warning");
document.querySelectorAll('input[name="timeframe"]').forEach((radio) => {
    radio.addEventListener("change", (event) => {
        currentTimeframe = event.target.value;
        timeframeWarning.hidden = currentTimeframe === "seconds";
        // Hours/days start from a stationary attitude (see CLAUDE.md), so
        // switching to one drops whatever the live preview had spun up.
        if (currentTimeframe !== "seconds") satelliteMesh.quaternion.identity();
    });
});

initScene();
initWheelSpeedCharts();
