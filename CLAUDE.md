# Spacecraft Attitude Determination and Control Simulator (ADCS Sim)

Portfolio project for a second-year Aerospace Engineering student (software space
engineering pivot, target: GitHub + possibly resume). Priorities: correct physics,
clean incremental builds, working live demo. Not a game, not a toy animation, an
engineering tool with a defensible README.

## What this project is

A web-based simulator that shows a satellite tumbling in 3D, lets the user trigger
a reaction-wheel-based attitude correction, and visualises the wheel torque/spin
response including saturation behaviour.

## User inputs

- Initial tumble/angular velocity (x, y, z)
- Satellite mass
- Satellite dimensions (box/rectangular prism, NOT a sphere, see Physics decisions)
- Mass and radius of each reaction wheel (three wheels, one per axis)
- Max RPM of each wheel
- A single generalised external disturbance torque magnitude + direction
  (explicitly NOT a physically derived sum of radiation pressure, drag, gravity
  gradient etc, see Physics decisions)

## Outputs

- 3D graphic of the satellite tumbling, fixed at the centre of the screen,
  draggable to view from any angle
- A "Correct Attitude" button that triggers the control system
- Reaction wheel torque shown as rotation vectors in their respective planes
  around the satellite
- Speed (RPM) vs time graphs per reaction wheel
- Timeframe switch: seconds / hours / days (see Timeframe behaviour below)

## Physics decisions (already made, do not relitigate without flagging why)

- **Box, not sphere.** A sphere has equal moment of inertia on all axes, which
  kills the interesting tumble dynamics. Satellite is approximated as a
  rectangular prism so moment of inertia differs per axis.
- **Test case is arbitrary-axis spin, not single-axis spin.** For an asymmetric
  (box) rigid body, torque-free rotation about a non-principal axis produces
  torque-free precession (chaotic-looking tumble even with zero external
  torque). This is correct physics, not a bug. The integrator must be verified
  against this general case, not just the simple single-axis case.
- **Reaction wheels are the actuator**, not thrusters or magnetorquers. Standard
  approach: internal momentum exchange via conservation of angular momentum,
  no propellant use, three wheels on orthogonal axes for full 3-axis control.
- **Wheel state is coupled to body state.** Wheel spin rate is its own state
  variable alongside the body's angular velocity, not just an output torque
  computed in isolation. When verifying in matplotlib, plot both together to
  catch angular-momentum bookkeeping errors.
- **Saturation logic:** each wheel has a max RPM (angular momentum limit). Once
  a wheel hits max RPM, it can no longer supply correction torque on that axis.
  Any further disturbance torque on a saturated axis causes the satellite body
  to actually accelerate (not drift at constant rate) in that axis, since
  nothing is opposing the disturbance anymore. Flag as "saturated" when hit.
- **External disturbance torque is a simplified stand-in**, a single
  user-supplied magnitude + direction representing the combined effect of solar
  radiation pressure, atmospheric drag, gravity gradient etc. Properly deriving
  these from orbital/geometric parameters is out of scope and should be stated
  as such in the README, not silently glossed over.
- **Correction torque output is the reaction torque exerted on the body by
  wheel spin-up**, not "torque applied to the wheels". Keep this distinction
  precise in code comments, plots, and any README/interview explanation.

## Timeframe behaviour

- **Seconds:** initial tumble is present, correction happens fast, no external
  disturbance modelled (correction timescale vastly faster than disturbance
  accumulation timescale, so combining them in one continuous run is not
  physically meaningful).
- **Hours/days:** satellite starts stationary (no initial tumble, it's already
  been corrected too fast to notice at this timescale). External disturbance
  torque is active here. Wheel spin rate climbs steadily until saturation, at
  which point the body starts accelerating in the saturated axis/axes.
- These are two separate regimes with different starting assumptions, by
  design. This is a deliberate simplification, not a shortcut to hide.

## Architecture

Two separate services, not a monolith:

- **`frontend/`** — static HTML/CSS/JS, Three.js for 3D rendering and drag
  controls. Deployed to GitHub Pages (free, static hosting only, cannot run
  Python).
- **`backend/`** — Python (Flask or FastAPI), runs the actual physics
  (scipy: rigid body dynamics, PID controller, reaction wheel + saturation
  logic). Deployed to a service that runs Python continuously (Render/
  Railway/Fly.io), NOT GitHub Pages.

Frontend and backend talk over a REST API: `script.js` sends user inputs as
JSON via `fetch` to a backend endpoint (e.g. `/simulate`), backend runs the
simulation and returns a time series of orientation states (quaternions) plus
wheel torque/spin data as JSON, frontend animates the 3D satellite through
those states and renders the torque/time graphs.

**CORS must be enabled on the backend** (`flask-cors` or equivalent) since
frontend and backend are on different domains once deployed. Set this up
early, not after hitting the silent-failure bug.

The backend's live URL is only known after first deployment. `script.js` needs
a config value for this that gets updated once the backend is deployed.

## File structure

```
adcs-sim/
├── frontend/
│   ├── index.html
│   ├── style.css
│   └── script.js
├── backend/
│   ├── app.py          # Flask entry point, API routes
│   ├── dynamics.py      # rigid body dynamics, moment of inertia, integration
│   ├── controller.py    # PID controller, reaction wheel + saturation logic
│   └── requirements.txt
├── .gitignore
├── README.md
└── CLAUDE.md
```

## Preferred build order (incremental, GUI-alongside-logic, not physics-then-GUI)

Core project:

1. Simplest possible screen: a 3D satellite (box) at the centre, draggable to
   view all sides. No physics yet.
2. Take user inputs, store as variables (frontend collects them, no backend
   wiring yet or minimal wiring just to pass values through).
3. Box-based rigid body dynamics using initial angular velocities. Satellite
   should visibly tumble according to the arbitrary-axis test case (precession
   included, not just simple single-axis spin). No reaction wheels yet.
4. Code the "Correct Attitude" button (wiring, not full control logic yet).
5. Reaction wheel dynamics and applied torque. Verify with matplotlib plots
   FIRST (plot body angular velocity and wheel spin rate together) before
   touching the 3D visualisation. Do not skip straight to 3D with unverified
   physics.
6. Once verified, output the velocity/orientation progression over time to the
   3D satellite, connected to the "Correct Attitude" button.

After core project is working:

7. Add seconds/hours/days timeframe switch. Include a clear warning in the UI
   that initial tumble is not modelled in hours/days (corrected too fast to
   notice at that timescale) and that the satellite starts stationary in those
   modes.
8. Model external disturbance torque in hours/days timeframes.
9. Apply reaction wheel corrections against that disturbance.
10. Apply saturation logic: wheel caps at max RPM, flags as saturated, body
    begins accelerating (not just drifting) in the saturated axis once no
    correction torque is available there.

## User conventions (apply throughout)

- Australian/British spelling in all code comments, README, and UI text
  (modelling, colour, optimise, etc).
- No em-dashes anywhere, in code comments, README, or generated text.
- When editing existing files, prefer targeted diffs over full rewrites where
  practical, mirroring how this project's owner works elsewhere.
- Any numeric claim or figure that might end up quoted in a README, resume
  bullet or interview should be physically defensible, don't let approximate
  or placeholder values silently become "the number" without flagging that
  they're placeholders.
