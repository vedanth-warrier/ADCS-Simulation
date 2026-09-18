"""PID controller and reaction wheel model, including saturation logic.

See CLAUDE.md Physics decisions: correction torque is the reaction torque on
the body from wheel spin-up, not torque applied to the wheels. Saturation
occurs at each wheel's max RPM (angular momentum limit).
"""

import numpy as np


class ReactionWheel:
    """A single reaction wheel: mass, radius, max RPM, current spin rate."""

    def __init__(self, mass, radius, max_rpm):
        # TODO: store params, compute wheel moment of inertia, init spin rate
        pass

    def is_saturated(self):
        """Return whether this wheel is at its max angular momentum."""
        # TODO
        pass


class PIDController:
    """PID attitude controller producing a commanded correction torque."""

    def __init__(self, kp, ki, kd):
        # TODO: store gains, init error accumulator
        pass

    def compute_torque(self, attitude_error, dt):
        """Return the commanded correction torque for the current error.

        TODO: standard PID, then hand off to reaction wheels for
        saturation-limited torque.
        """
        pass
