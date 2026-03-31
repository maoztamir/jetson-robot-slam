"""GPS reader using pyserial.

Reads NMEA sentences from a serial GPS module in a background thread and
exposes the latest fix via ``get_fix()``.  Only GGA sentences are parsed
(they carry lat/lon/alt/fix quality/satellite count).

Typical usage::

    gps = GPSReader(port="/dev/ttyTHS1", baud=9600)
    gps.start()

    fix = gps.get_fix()   # returns None if no valid fix yet
    if fix:
        print(fix["latitude"], fix["longitude"])

    gps.stop()
"""

from __future__ import annotations

import logging
import threading
import time
from typing import Any, Dict, Optional

logger = logging.getLogger("robot.gps")


def _parse_nmea_gga(sentence: str) -> Optional[Dict[str, Any]]:
    """Parse a $GPGGA / $GNGGA NMEA sentence.

    Returns a dict with keys:
        latitude, longitude, altitude_m, fix_quality, num_satellites, hdop
    or None if the sentence is malformed or has no fix (fix_quality == 0).
    """
    try:
        # Strip checksum (*XX) if present
        if "*" in sentence:
            sentence = sentence[: sentence.index("*")]
        parts = sentence.split(",")
        # $GPGGA,hhmmss.ss,lat,N/S,lon,E/W,fix,sats,hdop,alt,M,...
        if len(parts) < 10:
            return None
        if not parts[0].endswith("GGA"):
            return None

        fix_quality = int(parts[6]) if parts[6] else 0
        if fix_quality == 0:
            return None  # no fix

        def _dms_to_dd(raw: str, hemi: str) -> float:
            """Convert NMEA ddmm.mmmm / dddmm.mmmm to decimal degrees."""
            if not raw:
                raise ValueError("empty coordinate")
            # degrees occupy first 2 (lat) or 3 (lon) digits
            dot = raw.index(".")
            deg_digits = dot - 2
            degrees = float(raw[:deg_digits])
            minutes = float(raw[deg_digits:])
            dd = degrees + minutes / 60.0
            if hemi in ("S", "W"):
                dd = -dd
            return dd

        lat = _dms_to_dd(parts[2], parts[3])
        lon = _dms_to_dd(parts[4], parts[5])
        num_sats = int(parts[7]) if parts[7] else 0
        hdop = float(parts[8]) if parts[8] else None
        alt = float(parts[9]) if parts[9] else None

        return {
            "latitude": round(lat, 7),
            "longitude": round(lon, 7),
            "altitude_m": round(alt, 2) if alt is not None else None,
            "fix_quality": fix_quality,
            "num_satellites": num_sats,
            "hdop": round(hdop, 2) if hdop is not None else None,
        }
    except Exception:
        return None


class GPSReader:
    """Background-thread serial GPS reader.

    Args:
        port:    Serial device path (e.g. ``/dev/ttyTHS1``).
        baud:    Baud rate (default 9600).
        timeout: Serial read timeout in seconds.
    """

    def __init__(
        self,
        port: str = "/dev/ttyTHS1",
        baud: int = 9600,
        timeout: float = 1.0,
    ) -> None:
        self._port = port
        self._baud = baud
        self._timeout = timeout
        self._fix: Optional[Dict[str, Any]] = None
        self._fix_ts: Optional[float] = None
        self._lock = threading.Lock()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._mock = False

    # ----------------------------------------------------------------- public

    def start(self) -> None:
        """Open the serial port and start the reader thread."""
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run, name="gps-reader", daemon=True
        )
        self._thread.start()
        logger.info("GPS reader started on %s @ %d baud", self._port, self._baud)

    def stop(self) -> None:
        """Signal the reader thread to stop and wait for it."""
        self._stop_event.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
            self._thread = None
        logger.info("GPS reader stopped")

    def get_fix(self) -> Optional[Dict[str, Any]]:
        """Return the latest GPS fix dict, or ``None`` if no fix available.

        The returned dict has keys: latitude, longitude, altitude_m,
        fix_quality, num_satellites, hdop, timestamp.
        """
        with self._lock:
            if self._fix is None:
                return None
            return dict(self._fix)

    # ---------------------------------------------------------------- private

    def _run(self) -> None:
        """Reader thread: open serial port and parse lines until stopped."""
        ser = None
        try:
            import serial  # type: ignore
        except ImportError:
            logger.error("pyserial not installed -- GPS disabled (mock mode)")
            self._mock = True
            return

        while not self._stop_event.is_set():
            try:
                ser = serial.Serial(self._port, self._baud, timeout=self._timeout)
                logger.info("GPS serial port open: %s", self._port)
                while not self._stop_event.is_set():
                    line = ser.readline().decode(errors="ignore").strip()
                    if not line:
                        continue
                    fix = _parse_nmea_gga(line)
                    if fix is not None:
                        fix["timestamp"] = round(time.time(), 4)
                        with self._lock:
                            self._fix = fix
                        logger.debug(
                            "GPS fix: lat=%.6f lon=%.6f alt=%.1fm sats=%d",
                            fix["latitude"],
                            fix["longitude"],
                            fix["altitude_m"] or 0.0,
                            fix["num_satellites"],
                        )
            except Exception as exc:
                logger.warning("GPS serial error: %s -- retrying in 5 s", exc)
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
                self._stop_event.wait(5.0)
            finally:
                if ser is not None:
                    try:
                        ser.close()
                    except Exception:
                        pass
                    ser = None
