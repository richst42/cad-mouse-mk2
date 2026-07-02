# Tuner App — Product Roadmap

The tuner currently covers connect → calibrate → tune → verify. This roadmap
plans what turns it from a calibration utility into the companion tool that
takes a DIY build from soldered board to daily driver — and keeps it healthy.

**Who it serves.** Three hats, usually the same person at different moments:

1. **The first-time builder** — just finished the Instructables build. Job:
   *"Did I build it right?"* Today the app assumes a working device; nothing
   helps someone whose sensor is dead or magnet is flipped.
2. **The daily driver** — uses it in Fusion/SolidWorks/Blender every day.
   Job: *"Make it feel exactly right, in every app I use."*
3. **The maintainer** — six months in. Job: *"Is my spring wearing out?
   Is my calibration still good?"* (This is the README's PETG-longevity
   concern turned into a feature.)

---

## Phase 1 — Trust & daily comfort (quick wins)

### 1.1 Hardware self-test wizard  *(builder)*
One click after assembly: checks each sensor responds and streams, die
temperatures are sane, rest noise floor is within expected range, baseline
field magnitude per sensor is plausible (catches missing/flipped/weak
magnets by sign and magnitude), buttons register, LED ring lights. Output is
a pass/fail checklist with plain-language fixes ("MAG2 sees no field —
check the magnet above the top-left sensor"). This is the single highest
value add for the Instructables audience, and it's almost entirely
composition of data the app already receives.

### 1.2 Profiles  *(daily driver)*
Named settings snapshots — "CAD precise", "Sculpt fast", "Demo" — saved as
JSON on the PC, one click to push to the device. Includes gains, dead zones,
curve, signs, LEDs; optionally the calibration matrix. Import/export means
builders can share known-good starting points in the community. (On-device
profile slots cycled by a button chord can come later; PC-side is 90% of the
value at 10% of the work.)

### 1.3 Settings backup / restore  *(all)*
`Save backup…` / `Restore…` for everything on the device, including the
calibration matrix. Protects a 2-minute calibration from a botched
experiment, and migrates settings across firmware updates. Pairs with 1.2
(a backup is just an unnamed profile + matrix).

### 1.4 Auto-reconnect & quality-of-life  *(all)*
Remember the last port and reconnect when the device reappears (unplug,
sleep, flash). Toast-style status instead of console-only messages. A
"device" header always showing firmware version, uptime, sample rate, and
HID report rate — the at-a-glance "is everything healthy" strip.

### 1.5 Issue-report bundle  *(community)*
One button → a zip with settings, calibration matrix, quality diagnostics,
firmware version, and a 10-second raw capture. Turns "it drifts sometimes"
GitHub issues into diagnosable ones, and feeds real-world data back into
motion-processing improvements.

---

## Phase 2 — Differentiators (what commercial tools don't do)

### 2.1 Spring health monitor  *(maintainer — the README issue, productized)*
The app already sees everything needed: rest-point drift rate (how hard
auto-rezero is working), full-scale deflection amplitude vs. the values
recorded at calibration (stiffness proxy), and return-to-center time.
Track these per session, store a local history, chart the trend, and alert:
*"Spring stiffness is down 18% since calibration — time to print a spare."*
This converts the PETG-longevity worry from a caveat in the README into a
managed, observable property. No firmware changes required.

### 2.2 Crosstalk report card  *(quality)*
Replace the manual per-axis recording with one guided 60-second sequence
that walks all six axes and produces a scored before/after report (worst
off-axis %, per-axis grades). Store history alongside 2.1 so recalibration
need is visible ("crosstalk crept from 4% to 11% since March").

### 2.3 Response-curve editor  *(daily driver)*
The single exponent becomes a per-axis draggable curve (3–4 control points,
monotonic spline) with the measured input→output mapping plotted behind it.
Firmware side: a small per-axis lookup table (e.g. 9 points, interpolated)
uploaded like the matrix — cheap on the RP2040 and strictly more expressive
than the exponent.

### 2.4 A/B compare  *(daily driver)*
Hold two settings snapshots, toggle with one key while moving the knob,
optional blind mode ("A or B?" without showing which). Feel tuning is
subjective; fast switching is how you converge honestly.

### 2.5 Temperature compensation  *(quality)*
A guided warm-up capture (10 min of idle logging, temps already streamed)
fits per-channel baseline-vs-temperature coefficients and uploads them.
Kills the warm-up drift that auto-rezero currently papers over. Small
firmware addition (9×2 coefficients applied to the baseline).

---

## Phase 3 — Big bets

### 3.1 Built-in 3D test scene  *(daily driver)*
Grow the preview cube into a mini viewport: orbit/pan/zoom a real model
(a STEP-ish sample or a duck) driven by the live 6DoF output, with the
same control mappings CAD packages use. Closes the loop — tune, *feel it
immediately*, without alt-tabbing to Fusion. This is the feature that makes
the tuner feel like a product rather than a utility.

### 3.2 Per-application auto-profiles  *(daily driver)*
Watch the foreground window (Win32) and push the matching profile when you
switch apps — precise in SolidWorks, fast in Blender, LEDs dimmed in a
dark room. Builds directly on 1.2; the daemon-ish behavior pairs with a
system-tray mode (launch minimized, tray menu for profile switching).

### 3.3 Button mapping & axis modes  *(daily driver)*
Map the two hardware buttons (and chords/long-press) to: axis locks
(translation-only / rotation-only — the most-used feature on commercial
space mice), zero, profile cycle, or keyboard shortcuts synthesized by the
app. Firmware exposes button events already; modes are a settings flag.

### 3.4 Session replay & algorithm sandbox  *(tinkerer)*
Load a recorded CSV and re-run it through candidate processing (different
matrix, curves, dead zones) with side-by-side output plots. This is also
the harness the dipole-model pose solver (fix-plan Phase 3) gets prototyped
in — the app becomes the R&D bench for the firmware's future.

### 3.5 Firmware manager  *(all)*
Show installed vs. latest release (GitHub releases API), changelog, and
one-click download-and-flash using the existing flasher. Requires the repo
to publish `.uf2` release artifacts — worth doing regardless.

---

## Deliberately not doing

- **3Dconnexion driver emulation** — making apps believe this is a
  SpaceMouse touches proprietary driver stacks and is a separate project
  with real compatibility risk. The HID multi-axis descriptor already
  works where generic 6DoF input is accepted; keep scope there for now.
- **Cross-platform UI (macOS/Linux)** — the core (protocol, calibration,
  linalg) is already portable by design and tested off-Windows; a GLFW port
  is straightforward *when demand shows up*, but Windows-first keeps
  velocity. CAD usage skews Windows heavily.
- **Cloud anything** — profiles and history stay local files; sharing is
  copy-a-JSON. Right-sized for a DIY community tool.

## Suggested order

| # | Feature | Effort | Firmware change? |
|---|---------|--------|------------------|
| 1 | Hardware self-test wizard (1.1) | M | no |
| 2 | Profiles + backup/restore (1.2, 1.3) | S | no |
| 3 | Auto-reconnect + health strip (1.4) | S | version in PING (S) |
| 4 | Crosstalk report card (2.2) | M | no |
| 5 | Spring health monitor (2.1) | M | no |
| 6 | Issue-report bundle (1.5) | S | no |
| 7 | A/B compare (2.4) | S | no |
| 8 | Response-curve editor (2.3) | M | LUT upload (M) |
| 9 | Temperature compensation (2.5) | M | coeff apply (S) |
| 10 | 3D test scene (3.1) | L | no |
| 11 | Button mapping / axis locks (3.3) | M | modes (M) |
| 12 | Per-app auto-profiles + tray (3.2) | M | no |
| 13 | Session replay sandbox (3.4) | L | no |
| 14 | Firmware manager (3.5) | M | release artifacts |

Phase 1 is mostly composition of existing data — a week of evenings for a
big jump in usefulness. Phase 2 items 2.1/2.2 are the ones nothing else on
the market does for a printed-spring device. Phase 3.1 is the flagship.
