#!/usr/bin/env python3
"""
Trajectory Map Viewer
=====================
Retrieves robot trajectory data from the RobotTrajectory DynamoDB table and
renders interactive trajectory plots.

Features
--------
- Select one or more IoT devices from the table
- Filter records by date / time range (UTC), with quick-preset buttons
- Plots SLAM-frame ENU coordinates converted to lat/lon, or real GPS if present
- Zoomable / pannable matplotlib canvas with a navigation toolbar
- Export the current view as a PNG

Dependencies
------------
    pip install boto3 matplotlib

AWS credentials must be configured in the environment (e.g. ~/.aws/credentials,
environment variables, or an IAM role on the running host).

Usage
-----
    python scripts/trajectory_viewer.py
    python scripts/trajectory_viewer.py --region us-east-2 --table RobotTrajectory
"""

from __future__ import annotations

import argparse
import math
import os
import sys
import threading
from datetime import datetime, timedelta, timezone
from decimal import Decimal
from tkinter import filedialog, messagebox
from typing import Dict, List, Optional, Set, Tuple
import tkinter as tk
from tkinter import ttk

import boto3
from boto3.dynamodb.conditions import Attr, Key

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
import matplotlib.patches as mpatches

# ---------------------------------------------------------------------------
# Defaults (override via CLI args or the Settings panel in the UI)
# ---------------------------------------------------------------------------
DEFAULT_REGION = "us-east-2"
DEFAULT_TABLE = "RobotTrajectory"
# SLAM ENU origin — matches the CloudFormation ORIGIN_LAT / ORIGIN_LON values
ORIGIN_LAT = 32.0853
ORIGIN_LON = 34.7818
METRES_PER_DEG_LAT = 111_320.0

# Colour palette for up to 10 simultaneous devices
DEVICE_COLORS = [
    "#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd",
    "#8c564b", "#e377c2", "#7f7f7f", "#bcbd22", "#17becf",
]

# ---------------------------------------------------------------------------
# Coordinate helpers
# ---------------------------------------------------------------------------

def slam_to_latlon(
    x_m: float,
    y_m: float,
    origin_lat: float = ORIGIN_LAT,
    origin_lon: float = ORIGIN_LON,
) -> Tuple[float, float]:
    """Convert SLAM ENU metres to WGS-84 (lat, lon) decimal degrees."""
    lat = origin_lat + y_m / METRES_PER_DEG_LAT
    metres_per_deg_lon = METRES_PER_DEG_LAT * math.cos(math.radians(origin_lat))
    lon = origin_lon + (x_m / metres_per_deg_lon if metres_per_deg_lon > 1.0 else 0.0)
    return lat, lon


def _to_float(val) -> float:
    """Safely convert Decimal or numeric string to float."""
    return float(val)


def extract_latlon(item: dict, prefer_gps: bool = True) -> Optional[Tuple[float, float]]:
    """
    Extract (lat, lon) from a DynamoDB item (already deserialised by boto3).

    Returns None if the item lacks usable position data.
    """
    try:
        gps = item.get("gps")
        if prefer_gps and gps and "lat" in gps and "lon" in gps:
            return _to_float(gps["lat"]), _to_float(gps["lon"])

        pos = item.get("position", {})
        if "x" in pos and "y" in pos:
            return slam_to_latlon(_to_float(pos["x"]), _to_float(pos["y"]))
    except (TypeError, ValueError, KeyError):
        pass
    return None


# ---------------------------------------------------------------------------
# Reusable date+time entry composite widget
# ---------------------------------------------------------------------------

class DateTimeEntry(ttk.Frame):
    """A paired date entry (YYYY-MM-DD) and time entry (HH:MM:SS)."""

    def __init__(self, parent, initial: datetime | None = None, **kwargs):
        super().__init__(parent, **kwargs)
        if initial is None:
            initial = datetime.now(timezone.utc)
        self._date_var = tk.StringVar(value=initial.strftime("%Y-%m-%d"))
        self._time_var = tk.StringVar(value=initial.strftime("%H:%M:%S"))

        ttk.Entry(self, textvariable=self._date_var, width=11).pack(side=tk.LEFT)
        ttk.Label(self, text=" ").pack(side=tk.LEFT)
        ttk.Entry(self, textvariable=self._time_var, width=9).pack(side=tk.LEFT)

    # ---- public API -------------------------------------------------------

    def set(self, dt: datetime) -> None:
        self._date_var.set(dt.strftime("%Y-%m-%d"))
        self._time_var.set(dt.strftime("%H:%M:%S"))

    def get_datetime(self) -> datetime:
        """Parse entries and return a timezone-aware (UTC) datetime."""
        raw = f"{self._date_var.get().strip()} {self._time_var.get().strip()}"
        dt = datetime.strptime(raw, "%Y-%m-%d %H:%M:%S")
        return dt.replace(tzinfo=timezone.utc)

    def get_timestamp(self) -> float:
        return self.get_datetime().timestamp()


# ---------------------------------------------------------------------------
# Main application window
# ---------------------------------------------------------------------------

class TrajectoryViewer(tk.Tk):
    def __init__(self, region: str, table: str):
        super().__init__()
        self._region = region
        self._table_name = table

        self.title("Robot Trajectory Map Viewer")
        self.geometry("1300x820")
        self.minsize(960, 640)

        self._build_ui()
        # Kick off the device scan right away in the background
        self._refresh_devices()

    # =========================================================================
    # UI construction
    # =========================================================================

    def _build_ui(self) -> None:
        # Horizontal pane: left = controls, right = map
        paned = ttk.PanedWindow(self, orient=tk.HORIZONTAL)
        paned.pack(fill=tk.BOTH, expand=True, padx=4, pady=4)

        left_frame = ttk.Frame(paned, padding=(8, 8))
        paned.add(left_frame, weight=0)

        right_frame = ttk.Frame(paned)
        paned.add(right_frame, weight=1)

        self._build_controls(left_frame)
        self._build_map(right_frame)

    # ── Left control panel ────────────────────────────────────────────────

    def _build_controls(self, parent: ttk.Frame) -> None:
        row = 0

        # ── AWS config ──────────────────────────────────────────────────────
        ttk.Label(parent, text="AWS Configuration",
                  font=("", 10, "bold")).grid(row=row, column=0, columnspan=2,
                                               sticky=tk.W, pady=(0, 4))
        row += 1

        ttk.Label(parent, text="Region:").grid(row=row, column=0, sticky=tk.W)
        self._region_var = tk.StringVar(value=self._region)
        ttk.Entry(parent, textvariable=self._region_var, width=22).grid(
            row=row, column=1, sticky=tk.EW, padx=(6, 0))
        row += 1

        ttk.Label(parent, text="Table:").grid(row=row, column=0, sticky=tk.W,
                                              pady=(4, 0))
        self._table_var = tk.StringVar(value=self._table_name)
        ttk.Entry(parent, textvariable=self._table_var, width=22).grid(
            row=row, column=1, sticky=tk.EW, padx=(6, 0), pady=(4, 0))
        row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2,
                                   sticky=tk.EW, pady=8)
        row += 1

        # ── IoT device list ─────────────────────────────────────────────────
        ttk.Label(parent, text="IoT Devices",
                  font=("", 10, "bold")).grid(row=row, column=0, columnspan=2,
                                               sticky=tk.W)
        row += 1

        lb_frame = ttk.Frame(parent)
        lb_frame.grid(row=row, column=0, columnspan=2, sticky=tk.NSEW,
                      pady=(4, 0))
        lb_frame.rowconfigure(0, weight=1)
        lb_frame.columnconfigure(0, weight=1)

        sb = ttk.Scrollbar(lb_frame, orient=tk.VERTICAL)
        sb.grid(row=0, column=1, sticky=tk.NS)
        self._device_lb = tk.Listbox(
            lb_frame, selectmode=tk.MULTIPLE, height=8,
            yscrollcommand=sb.set, exportselection=False,
            activestyle="dotbox",
        )
        self._device_lb.grid(row=0, column=0, sticky=tk.NSEW)
        sb.config(command=self._device_lb.yview)
        row += 1

        btn_row = ttk.Frame(parent)
        btn_row.grid(row=row, column=0, columnspan=2, sticky=tk.W, pady=4)
        ttk.Button(btn_row, text="All", width=5,
                   command=lambda: self._device_lb.selection_set(
                       0, tk.END)).pack(side=tk.LEFT)
        ttk.Button(btn_row, text="None", width=5,
                   command=lambda: self._device_lb.selection_clear(
                       0, tk.END)).pack(side=tk.LEFT, padx=2)
        ttk.Button(btn_row, text="Refresh", width=8,
                   command=self._refresh_devices).pack(side=tk.LEFT, padx=2)
        row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2,
                                   sticky=tk.EW, pady=8)
        row += 1

        # ── Date / time range ───────────────────────────────────────────────
        ttk.Label(parent, text="Date / Time Range (UTC)",
                  font=("", 10, "bold")).grid(row=row, column=0, columnspan=2,
                                               sticky=tk.W)
        row += 1

        now = datetime.now(timezone.utc)

        ttk.Label(parent, text="From:").grid(row=row, column=0, sticky=tk.W,
                                             pady=(6, 0))
        row += 1
        self._from_dt = DateTimeEntry(parent, initial=now - timedelta(hours=24))
        self._from_dt.grid(row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1

        ttk.Label(parent, text="To:").grid(row=row, column=0, sticky=tk.W,
                                           pady=(6, 0))
        row += 1
        self._to_dt = DateTimeEntry(parent, initial=now)
        self._to_dt.grid(row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1

        ttk.Label(parent, text="Format: YYYY-MM-DD  HH:MM:SS",
                  foreground="gray").grid(row=row, column=0, columnspan=2,
                                         sticky=tk.W, pady=(2, 0))
        row += 1

        # Quick presets
        ttk.Label(parent, text="Quick range:").grid(
            row=row, column=0, columnspan=2, sticky=tk.W, pady=(8, 2))
        row += 1

        preset_frame = ttk.Frame(parent)
        preset_frame.grid(row=row, column=0, columnspan=2, sticky=tk.W)
        for label, hours in [("1 h", 1), ("6 h", 6), ("24 h", 24),
                              ("7 d", 168), ("30 d", 720)]:
            ttk.Button(
                preset_frame, text=label, width=5,
                command=lambda h=hours: self._apply_preset(h),
            ).pack(side=tk.LEFT, padx=1)
        row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2,
                                   sticky=tk.EW, pady=8)
        row += 1

        # ── Display options ──────────────────────────────────────────────────
        ttk.Label(parent, text="Display Options",
                  font=("", 10, "bold")).grid(row=row, column=0, columnspan=2,
                                               sticky=tk.W)
        row += 1

        self._prefer_gps_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(parent, text="Prefer GPS coordinates if available",
                        variable=self._prefer_gps_var).grid(
            row=row, column=0, columnspan=2, sticky=tk.W, pady=(4, 0))
        row += 1

        self._show_markers_var = tk.BooleanVar(value=True)
        ttk.Checkbutton(parent, text="Show start / end markers",
                        variable=self._show_markers_var).grid(
            row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1

        self._show_dots_var = tk.BooleanVar(value=False)
        ttk.Checkbutton(parent, text="Show individual data points",
                        variable=self._show_dots_var).grid(
            row=row, column=0, columnspan=2, sticky=tk.W)
        row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2,
                                   sticky=tk.EW, pady=8)
        row += 1

        # ── Action buttons ───────────────────────────────────────────────────
        self._query_btn = ttk.Button(parent, text="Load Trajectory",
                                     command=self._on_load)
        self._query_btn.grid(row=row, column=0, columnspan=2, sticky=tk.EW)
        row += 1

        ttk.Button(parent, text="Export PNG",
                   command=self._export_png).grid(
            row=row, column=0, columnspan=2, sticky=tk.EW, pady=(4, 0))
        row += 1

        ttk.Separator(parent).grid(row=row, column=0, columnspan=2,
                                   sticky=tk.EW, pady=8)
        row += 1

        # ── Status / stats ───────────────────────────────────────────────────
        self._status_var = tk.StringVar(value="Ready — refresh devices to begin.")
        ttk.Label(parent, textvariable=self._status_var, wraplength=245,
                  foreground="gray").grid(row=row, column=0, columnspan=2,
                                          sticky=tk.W)
        row += 1

        self._stats_var = tk.StringVar(value="")
        ttk.Label(parent, textvariable=self._stats_var, wraplength=245,
                  foreground="#004488").grid(row=row, column=0, columnspan=2,
                                             sticky=tk.W, pady=(4, 0))

        parent.columnconfigure(1, weight=1)

    # ── Right map panel ───────────────────────────────────────────────────

    def _build_map(self, parent: ttk.Frame) -> None:
        self._fig = Figure(figsize=(9, 7), dpi=100)
        self._ax = self._fig.add_subplot(111)
        self._init_axes()
        self._fig.tight_layout(pad=2)

        self._canvas = FigureCanvasTkAgg(self._fig, master=parent)
        self._canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        toolbar_frame = ttk.Frame(parent)
        toolbar_frame.pack(fill=tk.X)
        NavigationToolbar2Tk(self._canvas, toolbar_frame)

        self._canvas.draw()

    # =========================================================================
    # Helpers
    # =========================================================================

    def _init_axes(self) -> None:
        self._ax.clear()
        self._ax.set_xlabel("Longitude (°)")
        self._ax.set_ylabel("Latitude (°)")
        self._ax.set_title(
            "Select devices and a time range, then click  Load Trajectory",
            fontsize=9,
        )
        self._ax.grid(True, alpha=0.3, linestyle="--")

    def _get_table(self):
        return boto3.resource(
            "dynamodb", region_name=self._region_var.get()
        ).Table(self._table_var.get())

    def _apply_preset(self, hours: int) -> None:
        now = datetime.now(timezone.utc)
        self._from_dt.set(now - timedelta(hours=hours))
        self._to_dt.set(now)

    def _set_status(self, msg: str) -> None:
        self._status_var.set(msg)

    # =========================================================================
    # Device scanning
    # =========================================================================

    def _refresh_devices(self) -> None:
        self._set_status("Scanning table for devices…")
        threading.Thread(target=self._scan_devices_bg, daemon=True).start()

    def _scan_devices_bg(self) -> None:
        try:
            table = self._get_table()
            devices: Set[str] = set()
            kwargs: dict = {"ProjectionExpression": "device_id"}
            while True:
                resp = table.scan(**kwargs)
                for item in resp.get("Items", []):
                    did = item.get("device_id")
                    if did:
                        devices.add(str(did))
                lek = resp.get("LastEvaluatedKey")
                if not lek:
                    break
                kwargs["ExclusiveStartKey"] = lek
            self.after(0, self._populate_devices, sorted(devices))
        except Exception as exc:  # noqa: BLE001
            self.after(0, self._set_status, f"Error scanning devices: {exc}")

    def _populate_devices(self, devices: List[str]) -> None:
        self._device_lb.delete(0, tk.END)
        for d in devices:
            self._device_lb.insert(tk.END, d)
        self._device_lb.selection_set(0, tk.END)
        if devices:
            self._set_status(f"Found {len(devices)} device(s). Choose a range and load.")
        else:
            self._set_status("No devices found in the table.")

    # =========================================================================
    # Query + plot
    # =========================================================================

    def _on_load(self) -> None:
        selected_idx = self._device_lb.curselection()
        if not selected_idx:
            messagebox.showwarning("No device selected",
                                   "Please select at least one device from the list.")
            return

        devices: List[str] = [self._device_lb.get(i) for i in selected_idx]

        try:
            ts_from = self._from_dt.get_timestamp()
            ts_to = self._to_dt.get_timestamp()
        except ValueError as exc:
            messagebox.showerror("Invalid date / time",
                                 f"Use  YYYY-MM-DD  HH:MM:SS  (UTC).\n\n{exc}")
            return

        if ts_from >= ts_to:
            messagebox.showerror("Invalid range",
                                 "'From' must be earlier than 'To'.")
            return

        self._query_btn.config(state=tk.DISABLED)
        self._stats_var.set("")
        self._set_status(f"Querying {len(devices)} device(s)…")
        threading.Thread(
            target=self._query_bg,
            args=(devices, ts_from, ts_to),
            daemon=True,
        ).start()

    def _query_bg(self, devices: List[str],
                  ts_from: float, ts_to: float) -> None:
        try:
            table = self._get_table()
            all_data: Dict[str, List[dict]] = {}

            # The DynamoDB sort key `timestamp` is the SLAM monotonic clock
            # (a small float like 150.34 s since robot boot), NOT a Unix
            # timestamp.  The IoT rule adds `ingested_at` as Unix milliseconds
            # via  SELECT *, timestamp() AS ingested_at.
            # So we query by device_id only and filter on ingested_at.
            ingested_from_ms = int(ts_from * 1000)
            ingested_to_ms   = int(ts_to   * 1000)
            fe = Attr("ingested_at").between(ingested_from_ms, ingested_to_ms)

            for device_id in devices:
                items: List[dict] = []
                kce = Key("device_id").eq(device_id)
                resp = table.query(
                    KeyConditionExpression=kce,
                    FilterExpression=fe,
                    ScanIndexForward=True,
                )
                items.extend(resp.get("Items", []))

                # Paginate — FilterExpression is applied after reading each
                # page, so LastEvaluatedKey must be followed even when the
                # current page returned zero matching items.
                while resp.get("LastEvaluatedKey"):
                    resp = table.query(
                        KeyConditionExpression=kce,
                        FilterExpression=fe,
                        ExclusiveStartKey=resp["LastEvaluatedKey"],
                        ScanIndexForward=True,
                    )
                    items.extend(resp.get("Items", []))

                all_data[device_id] = items
                self.after(0, self._set_status,
                           f"Retrieved {len(items)} records for '{device_id}'…")

            self.after(0, self._plot, all_data, ts_from, ts_to)

        except Exception as exc:  # noqa: BLE001
            self.after(0, self._query_failed, str(exc))

    def _query_failed(self, msg: str) -> None:
        self._query_btn.config(state=tk.NORMAL)
        self._set_status(f"Error: {msg}")
        messagebox.showerror("Query failed", msg)

    # ── Plotting ──────────────────────────────────────────────────────────

    def _plot(self, all_data: Dict[str, List[dict]],
              ts_from: float, ts_to: float) -> None:
        self._init_axes()

        prefer_gps = self._prefer_gps_var.get()
        show_markers = self._show_markers_var.get()
        show_dots = self._show_dots_var.get()

        legend_handles: List = []
        total_plotted = 0
        record_counts: List[str] = []

        for idx, (device_id, items) in enumerate(all_data.items()):
            color = DEVICE_COLORS[idx % len(DEVICE_COLORS)]
            record_counts.append(f"{device_id}: {len(items)}")

            # Extract coordinates and timestamps
            pts: List[Tuple[float, float, datetime]] = []
            for item in items:
                ll = extract_latlon(item, prefer_gps=prefer_gps)
                if ll is None:
                    continue
                lat, lon = ll
                try:
                    # `ingested_at` is Unix milliseconds (added by IoT rule).
                    # `timestamp` is the SLAM monotonic clock — not wall time.
                    ingested_ms = _to_float(item["ingested_at"])
                    ts_dt = datetime.fromtimestamp(
                        ingested_ms / 1000.0, tz=timezone.utc
                    )
                except (KeyError, TypeError, ValueError):
                    ts_dt = datetime.fromtimestamp(0, tz=timezone.utc)
                pts.append((lat, lon, ts_dt))

            if not pts:
                legend_handles.append(
                    mpatches.Patch(color=color,
                                   label=f"{device_id}  (no plottable data)")
                )
                continue

            lats = [p[0] for p in pts]
            lons = [p[1] for p in pts]
            total_plotted += len(pts)

            # Trajectory line
            self._ax.plot(lons, lats, "-", color=color,
                          linewidth=1.8, alpha=0.85, zorder=2)

            # Individual data points
            if show_dots:
                self._ax.plot(lons, lats, ".", color=color,
                              markersize=3, alpha=0.45, zorder=3)

            # Start / end markers
            if show_markers:
                # Start: filled circle
                self._ax.plot(
                    lons[0], lats[0], "o", color=color, markersize=9,
                    markeredgecolor="white", markeredgewidth=1.5, zorder=5,
                )
                # End: filled star
                self._ax.plot(
                    lons[-1], lats[-1], "*", color=color, markersize=14,
                    markeredgecolor="white", markeredgewidth=1.0, zorder=5,
                )
                # Timestamp labels
                self._ax.annotate(
                    pts[0][2].strftime("%H:%M:%S"),
                    (lons[0], lats[0]),
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=7, color=color, zorder=6,
                )
                self._ax.annotate(
                    pts[-1][2].strftime("%H:%M:%S"),
                    (lons[-1], lats[-1]),
                    textcoords="offset points", xytext=(6, 4),
                    fontsize=7, color=color, zorder=6,
                )

            legend_handles.append(
                mpatches.Patch(color=color,
                               label=f"{device_id}  ({len(pts)} pts)")
            )

        if legend_handles:
            self._ax.legend(handles=legend_handles, loc="best", fontsize=9,
                            framealpha=0.85)

        # Format title with the actual time window
        from_str = datetime.fromtimestamp(ts_from, tz=timezone.utc).strftime(
            "%Y-%m-%d %H:%M"
        )
        to_str = datetime.fromtimestamp(ts_to, tz=timezone.utc).strftime(
            "%Y-%m-%d %H:%M"
        )
        self._ax.set_title(
            f"Trajectory Map  —  {from_str} → {to_str} UTC",
            fontsize=10,
        )

        self._fig.tight_layout(pad=2)
        self._canvas.draw()

        self._set_status(f"Done. {total_plotted} points plotted across "
                         f"{len(all_data)} device(s).")
        self._stats_var.set("Records fetched:\n" + "  |  ".join(record_counts))
        self._query_btn.config(state=tk.NORMAL)

    # =========================================================================
    # Export
    # =========================================================================

    def _export_png(self) -> None:
        path = filedialog.asksaveasfilename(
            title="Export trajectory map",
            defaultextension=".png",
            filetypes=[("PNG image", "*.png"), ("All files", "*.*")],
            initialfile="trajectory_map.png",
        )
        if path:
            self._fig.savefig(path, dpi=150, bbox_inches="tight")
            self._set_status(f"Exported → {path}")


# ---------------------------------------------------------------------------
# Entry point
# ---------------------------------------------------------------------------

def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Robot Trajectory Map Viewer")
    p.add_argument("--region", default=DEFAULT_REGION,
                   help="AWS region (default: %(default)s)")
    p.add_argument("--table", default=DEFAULT_TABLE,
                   help="DynamoDB table name (default: %(default)s)")
    return p.parse_args()


def main() -> None:
    args = _parse_args()

    # Verify required packages at startup so the user gets a clear message
    missing = []
    try:
        import boto3  # noqa: F401
    except ImportError:
        missing.append("boto3")
    try:
        import matplotlib  # noqa: F401
    except ImportError:
        missing.append("matplotlib")

    if missing:
        print(
            f"ERROR: missing package(s): {', '.join(missing)}\n"
            "Install them with:  pip install " + " ".join(missing),
            file=sys.stderr,
        )
        sys.exit(1)

    app = TrajectoryViewer(region=args.region, table=args.table)
    app.mainloop()


if __name__ == "__main__":
    main()