"""Simple 2-D IMU dead-reckoning integrator.

Integrates horizontal accelerometer data (X and Z axes) to produce a rough
position estimate when SLAM is unavailable.

Limitations
-----------
* Drift accumulates quickly — useful for relative motion over short runs,
  not for absolute localisation.
* Gravity is assumed to act on the Y axis (accel_y ≈ 1 g when level).
* Acceleration values are assumed to be in g units (as produced by the
  ICM-20948 / MPU-6050 drivers in this project).
* Velocity decay is applied when horizontal acceleration is near zero to
  prevent unbounded drift while the device is stationary.
"""

from __future__ import annotations

from typing import Dict, List, Tuple


class DeadReckoningIntegrator:
    """Euler-integrates IMU accel to estimate planar (X, Z) displacement.

    Coordinate convention matches ORB-SLAM3:
      X = right,  Y = up (gravity),  Z = forward

    The returned position uses the same (x, y=0, z) convention so it slots
    directly into the existing ``position`` field of trajectory payloads.
    """

    G: float = 9.81                    # m/s²
    STATIONARY_THRESH: float = 0.03    # g — |horiz accel| below this → stationary
    VELOCITY_DECAY: float = 3.0        # velocity halved every 1/VELOCITY_DECAY s

    def __init__(self) -> None:
        self._vx: float = 0.0
        self._vz: float = 0.0
        self._px: float = 0.0
        self._pz: float = 0.0
        self._last_ts: float | None = None

    # ---------------------------------------------------------------------- #

    def update(self, imu_samples: List[Dict]) -> Tuple[float, float, float]:
        """Integrate a batch of IMU samples.

        Args:
            imu_samples: List of IMUSample dicts (keys: timestamp,
                         accel_x, accel_y, accel_z, gyro_x/y/z).

        Returns:
            ``(x, y, z)`` position estimate in metres.  ``y`` is always 0
            (flat-surface assumption).
        """
        for s in imu_samples:
            self._step(s)
        return (round(self._px, 4), 0.0, round(self._pz, 4))

    @property
    def position(self) -> Tuple[float, float, float]:
        """Current position estimate ``(x, 0, z)`` in metres."""
        return (round(self._px, 4), 0.0, round(self._pz, 4))

    def reset(self) -> None:
        """Reset to origin."""
        self._vx = self._vz = 0.0
        self._px = self._pz = 0.0
        self._last_ts = None

    # ---------------------------------------------------------------------- #

    def _step(self, s: Dict) -> None:
        ts = float(s.get("timestamp", 0.0))

        if self._last_ts is None:
            self._last_ts = ts
            return

        dt = ts - self._last_ts
        if dt <= 0.0 or dt > 0.5:   # skip invalid / large gaps
            self._last_ts = ts
            return
        self._last_ts = ts

        # Horizontal accelerations in m/s² (Y = gravity, excluded)
        ax_g = float(s.get("accel_x", 0.0))
        az_g = float(s.get("accel_z", 0.0))
        ax = ax_g * self.G
        az = az_g * self.G

        # Velocity decay when approximately stationary
        horiz_g = (ax_g ** 2 + az_g ** 2) ** 0.5
        if horiz_g < self.STATIONARY_THRESH:
            decay = max(0.0, 1.0 - self.VELOCITY_DECAY * dt)
            self._vx *= decay
            self._vz *= decay

        # Euler integration: a → v → p
        self._vx += ax * dt
        self._vz += az * dt
        self._px += self._vx * dt
        self._pz += self._vz * dt
