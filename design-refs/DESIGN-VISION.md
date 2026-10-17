# Command — design system v0.4

Quiet laboratory instrument. Structure reads first. Empty and **unavailable** are honest states, not unfinished chrome.

This document is the visual and verbal contract for LabCommand. SwiftUI binds to `LabCommandCore` values; it does not invent point rates, bag growth, PTP, or Spark load.

## North star

Modern scientific DAQ / telescope control meets a pro-audio deck: deep charcoal, cool status accents, SF Pro hierarchy, high density that stays calm. An operator can live here for a two-hour walk without the panel shouting or lying.

The Mid-360 control loop is inviolable:

- Mac never talks to the Livox. SSH/Tailscale → `spark-capture.sh` → local `ws://localhost:8765` only. **Superseded** by the v1.0 addendum below and by #40 (D50, `docs/ROOM-MEMORY-PROTOCOL.md` §3.14): `scripts/spark-capture.sh` is deleted. The Mac opens two SSH tunnels (`RigService`: Foxglove `:8765`, command-center API `:8766`) and does not pipe a capture script.
- **Link → Stream → View → Record** and **Start path / Record / Stop all** stay the fastest path.
- **Stop all** is record → `docker stop -t 60` → close tunnel (FAST-LIO PCD flush).
- Bags are format-1 / CustomMsg + sidecar. `/Laser_map` is forbidden. **Superseded in part by #40 (D50):** the Mac provenance sidecar was `CaptureManifest`, and that file is deleted. This note does not replace the v0.4 format rule.
- Spark-offline is a first-class calm state. Missing keys render **unavailable**, never `0 Hz`.

## Before / after critique

**Before (v0.3 charcoal-steel shop console, `#141618`).** Four equal wells with a 6px pip and two lines of mono. The three actions sat in a flat row that looked the same armed or not (until a late opacity tweak). Landed data was one summary string. Blocked meant a yellow banner and a disabled button — no Diagnose, no recommended action, no session health. The palette was a single mid-charcoal slab: sidebar `#101214` almost identical to the canvas, surfaces `#1A1D20` with no instrument well, accent `#3D8BCC` used as a selection wash. Type was generic 13/11/12 with no tracked labels. Nothing was *wrong*, but nothing rewarded a long run: no telemetry hierarchy, no notebook, no timeline, no focus, no post-stop report. After twenty minutes it felt like a settings page with extra rectangles — a shop console that happened to own SSH, not a capture instrument.

**After (v0.4 instrument console).** Same three actions, same four stages, same gates. The rail is a single instrument strip: stage name, honest verb (Live / Idle / Blocked / —), one-line reason, then optional telemetry that prints **unavailable** when the Spark did not send the key. Session Health is a band + reason list, never a fake KPI. Blocked cells expose **Diagnose** (exact reason + one recommended action). Below the loop: a mission timeline of real events and a session library over `~/Desktop/lidar-sessions`. Focus hides secondary chrome without removing Start / Record / Stop. Motion only follows real state changes. The room got quieter and denser — not louder.

## Palette (hex)

| Token | Hex | Use |
|---|---|---|
| `bg` | `#0B0D10` | App canvas (cooler, deeper than `#141618`) |
| `sidebar` | `#080A0C` | Nav column |
| `surface` | `#12161A` | Wells, rail cells |
| `surfaceRaised` | `#181E24` | Armed controls, selected chips |
| `well` | `#0E1216` | Inset log / library |
| `line` | `#242B32` | Hairlines |
| `lineStrong` | `#343C45` | Control strokes |
| `text` | `#E6EAEF` | Primary copy |
| `dim` | `#8B95A1` | Secondary |
| `faint` | `#5C6670` | Labels, unavailable |
| `accent` | `#4A8FB8` | Selection, Live stroke, rare emphasis (cool steel, not cyan neon) |
| `accentMuted` | `#4A8FB8` @ 14% | Selected chip fill |
| `good` | `#3FA37A` | Live pip |
| `warn` | `#C4A04A` | Banner, Watch band |
| `bad` | `#C45A5A` | Blocked, Stop |
| `idle` | `#4A545E` | Idle pip |
| `unknown` | `#3A424A` | Em-dash state |

No gradients, no glow, no neon. Status color is a 6px pip or a 1px cell stroke — never a filled billboard.

## Typography

SF Pro / system default. Mono is SF Mono / `.monospaced`.

| Role | Size | Weight | Tracking | Notes |
|---|---|---|---|---|
| Section / stage label | 10.5 | medium | +0.8 | Uppercase |
| Destination title | 15 | semibold | 0 | Chrome cap — do not exceed |
| Stage verb | 13 | semibold | 0 | Live / Idle / Blocked / — |
| Body | 13 | regular | 0 | |
| Control | 13 | semibold | 0 | Start path / Record / Stop all |
| Telemetry / IDs | 11 | regular mono | 0 | |
| Health band | 13 | semibold | 0 | Word, not a giant number |
| Menu-bar CAP | 11 | medium mono | 0 | |

Never display a live Hz / byte figure larger than 13pt. Never use display / rounded display faces.

## Status language

| State | Verb | Meaning |
|---|---|---|
| running | **Live** | Observed running (container, tunnel, bag, or carrier=1) |
| idle | **Idle** | Probed and off |
| blocked | **Blocked** | Known failure; Diagnose is offered |
| unknown | **—** | Not probed or key absent |

Telemetry missing → the word **unavailable**, never `0 Hz`, `0 B/s`, or a greyed fake sparkline.

Session Health bands (see `CaptureHealth`):

| Band | When |
|---|---|
| **Unavailable** | Spark up but too few stage signals to judge |
| **Blocked** | Spark down, NIC NO-CARRIER, or an explicit block reason |
| **Thin** | Link up, stream not live; few signals |
| **Watch** | Stream live but record/view/telemetry incomplete |
| **Ready** | Link + stream live and gates would allow a record |

A **score** (0–100) is shown only when enough *stage* signals exist. Offline snapshots have `score == nil` and `isHealthyLive == false`. Omitted JSON keys never become rates. Ready is not claimed from empty telemetry.

## Spacing and shape

| Token | Value |
|---|---|
| Sidebar width | 196 |
| Top bar | 40 |
| Page pad | 20 |
| Section gap | 20 |
| Rail gap | 8 |
| Radius | 7 |
| Control radius | 5 |
| Pip | 6 |

Density first: the four-stage rail is one row; health is a single strip above it; the three actions sit immediately under the rail (or under the Diagnose banner). Secondary surfaces (library, log) stack below. Focus removes sidebar + library + berths, not the rail or the three actions.

## Motion

- Color / opacity 180ms ease-out on real state changes only.
- No spring, no bounce, no layout shuffle for decoration.
- A Live pip may breathe (1.6s opacity 1.0 ↔ 0.55) **only** while that stage is actually `.running`.
- Disabled controls: opacity 0.38, no pulse.
- Focus chrome fade: 200ms opacity. Do not animate numbers.

## Banned (still)

- Thesis / manifesto / wedge / ICP copy in chrome
- Fake metrics, KPI cards, activity feeds, invented Hz
- Neon HUD, gradient glows, “coming soon”
- Empty inspector columns
- Mixing in `lidar_record.py` or treating the Mac as a Livox client
- Painting Stream/Record **Live** when `ssh spark` failed

## Component patterns

- `InstrumentWell` — hairline + `#12161A` fill; title tracked; body mono or SF Pro 13.
- `StageCell` — label, verb+pip, detail, optional telemetry rows, Diagnose if blocked.
- `ActionRow` — Start path / Record / Stop all, always present in Capture and Focus.
- `HealthStrip` — band word + headline reason; score only if non-nil.
- `MissionLine` — `yyyy-MM-dd HH:mm:ss` + kind + text from `MissionLog`.
- `SessionRow` — file-derived bytes/points/PTP/tags; notes from disk.

## Keyboard and menu-bar

| Binding | Action |
|---|---|
| ⌘↩ | Start path |
| ⌘⇧R | Record |
| ⌘. | Stop all |
| ⌘R | Refresh (Capture / Devices) |
| ⌘1…5 | Capture / Devices / Research / Product / Settings |
| ⌘⇧F | Focus mode |
| ⌘⇧D | Diagnose primary block |

Menu-bar **CAP** shows honest Link/Stream/View/Record verbs and the health band. It never prints a point rate unless `CaptureTelemetry.pointRateHz` is non-nil. **Superseded by #40 (D50):** `struct CaptureTelemetry` is gone. The menu-bar extra prints a rate only when `StatusDots` has a non-nil `SensorReading.pointRateHz` and that sensor's state is `.streaming` (`LabCommandApp.rebuildStatusMenu`), formatted by `TelemetryFormat.hz`.

## Architecture (SwiftUI)

- `LabCommandCore` owns snapshot, gates, parser, health, Diagnose, session index, mission log, stop report, device recovery. Pure and testable.
- `AppState` is the only view model the chrome binds to. It probes via `CaptureController` (SSH + script + tunnel) and **assigns** parser output — it does not fill missing Hz.
- Views: `CaptureView` (rail + actions + log + compact library), `SessionLibraryView` (Research), `DevicesView` (probes + recovery), `CommandCenterView` (nav + Focus).
- Focus is `AppState.focusMode`. It does not change gates.

## v1.0 addendum — dual rig

Everything above stands. v1.0 changes the domain, not the visual contract:

- The four-stage rail becomes **Link → Drivers → Clocks → View → Record**,
  preceded by the **sensor duo** (Mid-360 / JT128 / Clocks cards).
- New band **Parked** (both sensors deliberately unpowered, infra healthy):
  rendered in the steel accent, never amber or red. An unplugged sensor is
  a gray pip labeled Dark with a plain-language line — parking the rig is
  an operator action, not a failure.
- Sensor verbs: **Dark / Powered / Live / —**. Powered means "answering,
  no points" and the detail must say what is knowable (standby vs driver),
  never guess.
- Data plane is the Spark command-center API over localhost:8766; the SSH
  script/parser path is gone (`RigService` owns the two tunnels). The
  honest-state rules transfer verbatim: absent JSON keys are nil →
  "unavailable"; a measured zero is a real zero and may alarm; the UI
  invents nothing the collector did not measure.

## v1.2 addendum — the pipeline graph

The Rig page's sensor duo became the **signal chain, drawn** (`PipelineGraph`):
sensor nodes → labeled wires → Spark node → topic wires → output nodes
(lidar-dual / Foxglove / bag writer), with an **audit strip** of per-hop
checks beneath. Rules:

- **Every wire is a real hop** with its actual protocol and port (UDP
  56301/56401, UDP 2368, TCP 9347 PTC, PTP 1588v2, the two ROS topics).
  No decorative connectors. Data wires point right; control wires (PTP,
  PTC) point left and are always dashed.
- Wire color follows measured state only: good = points flowing, warn =
  powered/no data, dashed idle = parked, steel = control path alive.
- Node spec lines (`RigSpec.nodeSummary`) are **static hardware facts** —
  they never blend with telemetry, and full dossiers live on the Hardware
  page (`RigSpec.dossier`), cautions rendered in warn.
- The audit strip (`PipelineAudit.checks`) has five verdicts: pass / watch
  / fail / **pending** / unavailable. Pending is steel, not amber — it
  means "needs the sensors powered", and a parked healthy rig shows zero
  fails. Loss counters judge **deltas**, never lifetime totals (the NIC
  carries 1.03M historical rx_mac_missed from before rmem tuning).
- A probe the collector didn't send renders unavailable — an old collector
  must never look like a passing audit.
