"""2-D IMU dead-reckoning integrator with gravity subtraction and ZUPT.

The previous naive integrator failed because a ~4° device tilt produces a
constant ~0.07 g on the Z axis that, when integrated twice, yields ~1 km of
drift in 60 seconds.

This version fixes that with two techniques:

1. **Gravity estimation** — an exponential moving-average (EMA) of the raw
   accelerometer signal estimates the gravity vector in the device frame.
   Because real motion accelerations are brief, the EMA converges to gravity
   and is subtracted before integration, leaving only motion-induced accel.

2. **Zero-velocity update (ZUPT)** — whenever the residual acceleration AND
   the gyro magnitude are both small (device is stationary), velocity is
   zeroed to prevent unbounded drift from noise accumulation.

Limitations
-----------
* Heading is not tracked — suitable for showing relative displacement on a
  flat surface when turns are small.
* Drift is still present during continuous motion; results are best for
  short runs (< 60 s between ZUPT events).
* Acceleration values must be in g units; gyro values in deg/s (as produced
  by the ICM-20948 driver in this project).
"""

from __future__ import annotations

import math
from typing import Dict, List, Optional, Tuple


class DeadReckoningIntegrator:
    """Integrates IMU accel to estimate 2-D planar (X, Z) displacement.

    Coordinate convention matches ORB-SLAM3:
      X = right,  Y = up (gravity axis),  Z = forward

    The returned position ``(x, 0, z)`` slots directly into the existing
    ``position`` payload field used by the web dashboard.
    """

    G: float = 9.81                  # m/s²

    # EMA coefficient for gravity estimate.
    # Higher = slower adaptation (better noise rejection, slower tilt tracking).
    GRAVITY_ALPHA: float = 0.98

    # ZUPT: zero velocity when BOTH conditions hold for a sample.
    ZUPT_ACCEL_THRESH: float = 0.04  # g  — residual |accel - gravity|
    ZUPT_GYRO_THRESH:  float = 3.0   # deg/s — total gyro magnitude

    def __init__(self) -> None:
        self._vx: float = 0.0
        self._vz: float = 0.0
        self._px: float = 0.0
        self._pz: float = 0.0
        self._last_ts: Optional[float] = None

        # Gravity EMA — initialised to [0, 1, 0] (Y is up, device level)
        self._grav_x: float = 0.0
        self._grav_y: float = 1.0
        self._grav_z: float = 0.0

    # ------------------------------------------------------------------ #

    def update(self, imu_samples: List[Dict]) -> Tuple[float, float, float]:
        """Integrate a batch of IMU samples and return position estimate.

        Args:
            imu_samples: List of IMUSample dicts with keys
                         timestamp, accel_x/y/z (g), gyro_x/y/z (deg/s).

        Returns:
            ``(x, 0.0, z)`` position in metres from the start point.
        """
        for s in imu_samples:
            self._step(s)
        return (round(self._px, 4), 0.0, round(self._pz, 4))

    @property
    def position(self) -> Tuple[float, float, float]:
        """Current position estimate ``(x, 0, z)`` in metres."""
        return (round(self._px, 4), 0.0, round(self._pz, 4))

    def reset(self) -> None:
        """Reset to origin (called at the start of each run)."""
        self._vx = self._vz = 0.0
        self._px = self._pz = 0.0
        self._last_ts = None
        self._grav_x = 0.0
        self._grav_y = 1.0
        self._grav_z = 0.0

    # ------------------------------------------------------------------ #

    def _step(self, s: Dict) -> None:
        ts  = float(s.get("timestamp", 0.0))
        ax  = float(s.get("accel_x",   0.0))   # g
        ay  = float(s.get("accel_y",   0.0))   # g
        az  = float(s.get("accel_z",   0.0))   # g
        gx  = float(s.get("gyro_x",    0.0))   # deg/s
        gy  = float(s.get("gyro_y",    0.0))   # deg/s
        gz  = float(s.get("gyro_z",    0.0))   # deg/s

        if self._last_ts is None:
            self._last_ts = ts
            # Seed gravity from first sample
            self._grav_x, self._grav_y, self._grav_z = ax, ay, az
            return

        dt = ts - self._last_ts
        if dt <= 0.0 or dt > 0.5:          # skip invalid / large gaps
            self._last_ts = ts
            return
        self._last_ts = ts

        # --- 1. Update gravity estimate (EMA) ----------------------------
        a = self.GRAVITY_ALPHA
        self._grav_x = a * self._grav_x + (1 - a) * ax
        self._grav_y = a * self._grav_y + (1 - a) * ay
        self._grav_z = a * self._grav_z + (1 - a) * az

        # --- 2. Subtract gravity → linear acceleration (g units) ---------
        lax = ax - self._grav_x
        laz = az - self._grav_z

        # --- 3. ZUPT check -----------------------------------------------
        residual_g  = math.sqrt(lax ** 2 + laz ** 2)
        gyro_mag    = math.sqrt(gx ** 2 + gy ** 2 + gz ** 2)

        if residual_g < self.ZUPT_ACCEL_THRESH and gyro_mag < self.ZUPT_GYRO_THRESH:
            # Device is stationary — zero velocity to prevent drift
            self._vx = 0.0
            self._vz = 0.0
            return

        # --- 4. Euler integration: accel → velocity → position -----------
        ax_ms2 = lax * self.G
        az_ms2 = laz * self.G

        self._vx += ax_ms2 * dt
        self._vz += az_ms2 * dt
        self._px += self._vx * dt
        self._pz += self._vz * dt
