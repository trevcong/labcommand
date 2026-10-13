# Command

macOS instrument console for the **dual-lidar rig** (Livox Mid-360 + Hesai
JT128) on the DGX Spark.

**v1.0 is a 100% rehaul.** The v0.x app drove a single Mid-360 by piping
`spark-capture.sh` over SSH and parsing its output. v1.0 binds to the
Spark's **command-center API** (`~/hesai_ws/command_center.py`, :8766 over
an SSH tunnel) — the same collector that serves the web panel and the
`lidar` CLI. One collector, one truth, three views.

What carried over unchanged: the design system (DESIGN-VISION.md), the
honest-state contract (missing keys print **unavailable**, a measured 0 Hz
is a real zero, "a running container is not a running stream"), the mission
log, session library, trajectory maps, focus mode, and the menu-bar extra.

What's new:

- **Two sensors, four honest states each**: Dark (unplugged is a normal,
  calm state — the rig reads **Parked**, not broken), Powered (answering,
  no points — standby or driver down), Live (with measured rates), and —
  (not probed).
- **JT128 Wake / Standby** on the sensor card — stops the motor and lasers
  for heat without unplugging (PTC 0x1C via the API).
- **Clocks card**: the ptp4l software master plus per-sensor clock offsets
  measured from IMU stamps. Knows that the Mid-360 slews ~10 ms/min after
  power-up while the JT128 steps instantly.
- **Watchdog aware**: plugging a sensor back in restarts the Spark stack
  automatically; the app just watches it happen and logs the event.
- **Dual recording**: one bag with both point clouds + both IMUs
  (`<ts>_DUAL_<code>_<tag>` on the Spark, ~2 GB/min when both stream).
- **5 s polling** — an HTTP GET over the tunnel, not an SSH round-trip, so
  the panel tracks reality at web-page speed.

## Open

```bash
cd ~/Desktop/hdc-spatial-memory/LabCommand
./build.sh
open ~/Applications/LabCommand.app
```

Menu bar extra: **LAB**. Capture menu: ⌘↩ start/restart rig, ⌘⇧R record,
⌘. stop, ⌘⇧D diagnose. View: ⌘⇧F focus, ⌘R refresh.

## Map

| Dest | ⌘ | Purpose |
|---|---|---|
| **Rig** | 1 | Sensor duo, transport, readout, path rail, take planner |
| **Hardware** | 2 | Spark / Tailscale / per-sensor cards + containers + watchdog events |
| **Takes** | 3 | Session library (search, compare, notes) |
| **Settings** | 4 | Host, probe interval, capture paths |

## What the buttons do

| Control | What happens |
|---|---|
| Start / Restart rig | Opens both SSH tunnels (8765 view, 8766 API), then `start`/`restart` on the API → `lidar-dual` container |
| Start recording | API `record` → `ros2 bag` of all four topics inside `lidar-dual` |
| Stop | Recording? `stop-record` (SIGINT + chown). Else `down` (60 s grace) + close tunnels |
| Wake / Standby | API `wake`/`standby` → JT128 PTC 0x1C |

The Spark side lives in `~/hesai_ws/` (`command_center.py`, `dual_up.sh`,
`ptp_up.sh`, `watchdog.sh`); the rig write-up is
`~/Desktop/accountable-robotics-co/DUAL-RIG.md`.

## Tests

```bash
swift run LabCommandCoreTestsRunner
```

Pins the decoder's honest rules, the gates (record needs points, not
containers), the Parked band, dual session naming, and the v0.4 design
contract (type cap, spacing tokens, no placeholder chrome).
