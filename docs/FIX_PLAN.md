# Fix Plan for Known Issues

This document plans fixes for the two main issues called out in the READMEs:

1. **Motion processing** ([firmware/README.md](../firmware/README.md)): axis bleed between the six axes, and an implicit assumption that sensor readings are linear with displacement (they are not).
2. **PETG spring longevity** ([README.md](../README.md)): concerns that the printed spring will creep or fatigue over time.

---

## Issue 1 — Motion processing (axis bleed + nonlinearity)

### Why it happens today

`MotionController::compute()` ([MotionController.cpp](../firmware/src/controllers/MotionController.cpp)) maps the nine baseline-subtracted field components to six axes with fixed, hand-derived linear combinations (averages for Tx/Ty/Tz, z-differences for Rx/Ry, a swirl sum for Rz). Two physical realities break this:

- **Nonlinearity.** The field of a magnet falls off roughly with 1/r³. A 1 mm displacement near rest produces a much smaller field delta than the same 1 mm at full deflection, so the response curve is progressively too hot at the extremes (or too dead near center, depending on gain).
- **Cross-coupling.** A pure motion on one axis changes field components that the other axes' formulas also read. Example: tilting the knob (Rx) moves the magnets laterally as well as vertically, so the Tx/Ty averages pick up signal; pushing straight down (Tz) changes x/y components near the sensors, so the Rz swirl term picks up signal. The fixed formulas have no way to cancel this because they ignore the actual per-unit response of each sensor channel to each motion.

There is also a smaller firmware-level artifact that *feels* like bleed: the dead zone is a hard gate (`filt_[i] = 0` inside the zone, raw pass-through outside, plus `hardZero()` after clamping). Output snaps from 0 to ≥ DEAD the instant an axis crosses the threshold, which makes incidental cross-axis leakage appear as sudden jumps.

### Fix plan (three phases, each independently shippable)

#### Phase 1 — Smooth response shaping (low risk, no recalibration needed)

Goal: eliminate the snap at the dead-zone edge and make small cross-axis leakage fade in gradually instead of jumping.

- Replace the hard dead zone with a **soft dead band**: `y' = sign(y) * max(0, |y| - dead)` then rescale so full deflection still reaches `AXIS_LIMIT`. Output ramps from 0 continuously.
- Filter **before** the dead band, and stop resetting `filt_[i]` to zero inside the zone — let the low-pass state decay naturally so entering/leaving the zone is continuous.
- Optionally add a per-axis **response curve** (e.g. `y' = y * |y|^k` with k ≈ 0.5–1.0, normalized to AXIS_LIMIT) as a crude first correction for the 1/r³ steepening, tunable in `Config.h`.

Touches: `MotionController.cpp/.h`, `Config.h`, firmware README (document new tunables).

#### Phase 2 — Empirical decoupling matrix (the real fix for bleed)

Goal: replace the hand-derived formulas with a calibration that measures how *this specific unit* responds, cancelling cross-coupling to first order.

- Model the small-displacement relationship as **ΔB ≈ J·p**, where ΔB is the 9-vector of field deltas, p the 6-vector pose, and J a 9×6 Jacobian.
- Add a **guided calibration mode** (extend `CalibratingState` or add a new state, LED-signalled): the user holds each pure motion at full deflection in turn (+Tx, −Tx, +Ty, … −Rz, 12 poses). Average the sensor deltas per pose to get the columns of J.
- Compute the **pseudo-inverse J⁺ (6×9)** on-device (a 6×6 solve via Gauss-Jordan is enough on the RP2040) and use `p = J⁺·ΔB` at runtime instead of the fixed formulas. Cross-coupling is cancelled by construction because J captures the real geometry, magnet strength, and assembly tolerances of the unit.
- **Persist the matrix** (LittleFS/EEPROM emulation on the XIAO RP2040) so guided calibration is a one-time setup; the existing quick zero-baseline calibration remains the per-boot routine.
- Normalize each axis by the measured full-deflection magnitude, which also makes the Phase-1 dead zones and curves consistent across axes and across builds.

Touches: `MotionController`, `SensorController` (calibration capture), `StateMachine`/states, `Config.h`, README.

#### Phase 3 — Model-based pose estimation (optional, replaces the pipeline)

The author notes the processing "may eventually be replaced entirely." The principled version:

- Fit the full 6-DoF pose each frame by **least-squares against a magnetic dipole model** of the knob's magnets versus the three known sensor positions (Gauss-Newton, seeded with the previous frame's pose; 3–5 iterations converge). This handles nonlinearity *and* coupling exactly, not just to first order.
- Prototype offline first: log raw telemetry (`TelemetryController` already streams it) while moving the knob, fit the model on a PC, and validate before porting to the RP2040. The RP2040 has no FPU, so profile; if too slow, fall back to Phase 2's linear map plus a lookup-table correction sampled from the model.

#### Bonus (cheap, high value): slow baseline auto-rezero + temperature compensation

Thermal drift and spring settling shift the rest point between manual calibrations. When all axes have been inside the dead zone for a few seconds, slowly pull `baseline_` toward the current reading (time constant ~10 s). This also masks gradual spring creep — see Issue 2.

The TLx493D sensors already report die temperature, and `SensorController::readRaw()` reads it on every frame and discards it ([SensorController.cpp:59-61](../firmware/src/controllers/SensorController.cpp)). Log temperature alongside field data during test captures; if the baseline correlates with temperature, add a simple linear temperature-compensation term per channel. Between auto-rezero and temperature compensation, the dead zones (`DEAD_T = 16`, `DEAD_R = 20`) can shrink substantially, restoring small-motion sensitivity.

Note on Phase 2 formulation: measuring the 9×6 Jacobian and pseudo-inverting it is equivalent to directly regressing a 6×9 matrix from sensor deltas to pose by least squares. The direct-regression form is preferable in practice because it extends naturally to nonlinear features (e.g. appending quadratic terms of the sensor deltas to the input vector) as a middle step between Phase 2 and the full model-based Phase 3. One practical wrinkle: humans cannot produce perfectly pure single-axis motions, so the guided capture should use *held* max-deflection poses (repeated a few times, averaged) rather than free motion sweeps.

### Suggested order & validation

1. Phase 1 + auto-rezero (small diffs, immediately better feel).
2. Phase 2 (fixes bleed properly; the biggest payoff).
3. Phase 3 only if Phase 2's first-order model still shows objectionable nonlinearity at large deflections.

Validate each phase with telemetry captures of pure single-axis motions: record all 6 outputs while exercising one axis; off-axis RMS relative to on-axis peak is the bleed metric. Target < 5 % after Phase 2 (currently visibly worse).

---

## Issue 2 — PETG spring longevity

### Why it's a concern

PETG creeps under sustained load and has modest fatigue life. The spring ([Spring (parametric).f3d](../enclosure/fusion/), STLs at 1.0 mm and 0.8 mm × 30°) is strained on every input and, worse, held deflected for seconds at a time during use. Layer lines act as crack initiators at the arm roots. Failure modes: gradual sag (rest point drifts, feel softens) then arm fracture.

### Fix plan

#### Short term — documentation and printing guidance (no redesign)

- Add a README/Instructables section on **material choice**: PC blend or nylon (PA12/PA-CF) substantially outperform PETG for creep and fatigue; annealed PETG is a middle ground. The parametric Fusion file already allows re-tuning thickness per material stiffness.
- Document **print orientation and settings** that maximize spring life (arms loaded along, not across, layer lines where geometry allows; 100 % infill; slow/cool for layer adhesion).
- Treat the spring as a **consumable**: recommend printing spares (it's a small part) and note the symptom of a sagging spring (rest-point drift needing frequent recalibration).

#### Medium term — design revision to lower stress

- **Travel limiters**: add hard stops in the knob/stem so the spring cannot be deflected past its elastic design range. Over-travel is the main accelerant of both creep and fracture; this is the single most effective mechanical change and is compatible with the existing spring.
- **Stress reduction in the parametric spring**: larger root fillets at the arm attachments, and/or more arms with lower per-arm strain, keeping total stiffness (and therefore sensor gain calibration) roughly constant.
- Publish the revised STLs alongside the current ones (as done for the 1.0/0.8 mm variants) so builders can choose.

#### Long term — the knob revision the author anticipated

If printed springs prove inadequate, revise the knob/stem to accept a **non-printed elastic element**: steel wave/compression springs, or silicone O-rings between stem and knob. Either gives effectively unlimited fatigue life; O-rings additionally add damping which improves the return-to-center feel. This is the "revision of the knob design" flagged in the README, and should reuse the same magnet holder and sensor geometry so no PCB or firmware changes are needed.

#### Firmware mitigation (ties into Issue 1)

The baseline auto-rezero from Issue 1 directly compensates gradual spring sag, extending the usable life of any spring by keeping the rest point centered without manual recalibration.

---

## Host tuning app (Windows, Dear ImGui)

A desktop companion app for live preview, calibration, and tuning. This becomes the test harness that quantifies crosstalk before/after each firmware phase, and removes the edit-`Config.h`-and-reflash loop entirely.

### Firmware side — serial protocol (prerequisite)

The current telemetry ([TelemetryController.cpp](../firmware/src/controllers/TelemetryController.cpp)) prints only the six *filtered* outputs as Teleplot-style text, every 5th frame. The app needs more, so extend the serial link with:

- **Streaming**: a compact line or binary-framed record per frame containing the raw 9-vector, die temperatures, baseline, pre-filter axis values, and final outputs. Selectable stream modes (off / outputs-only / full) via command.
- **Command channel** for runtime configuration: `get`/`set` for gains, dead zones, smoothing tau, response-curve exponent, sign flips; `save` to persist to flash (LittleFS); `cal` to trigger zero-calibration or start the guided 12-pose capture; `matrix` to upload a decoupling matrix computed on the PC.

Runtime-settable parameters persisted to flash are strongly preferable to regenerating firmware: tuning becomes an instant A/B comparison with no reflash and no toolchain on the tuning machine. As a fallback for source-level changes, the app can export a `Config.h` snippet matching the current slider state.

This means `Config.h` values become *defaults* loaded at boot and overridden by stored settings — a small refactor of `Config` from constants to a settings struct.

### App side

- **Stack**: Dear ImGui + ImPlot, Win32/DirectX 11 backend (the stock ImGui example scaffold), CMake, serial via Win32 `CreateFile`/overlapped I/O or libserialport, Eigen for the least-squares calibration solve. No other dependencies.
- **Views**:
  - Live strip charts of all raw channels, temperatures, and the six outputs (ImPlot).
  - 6DoF preview: bar meters plus a simple 3D-cube gizmo driven by the output pose — the "preview the results" view.
  - Crosstalk panel: user exercises one axis at a time; the app computes off-axis RMS vs. on-axis peak and renders a 6×6 heatmap. This is the before/after metric for Phases 1–3.
  - Calibration wizard: walks through the 12 held poses, records averaged deltas, solves the 6×9 matrix (optionally with quadratic features), previews the result live, then uploads and saves to flash.
  - Tuning panel: sliders for gains/dead zones/smoothing/curve bound to the serial command channel, with save-to-flash and export-`Config.h` buttons.
  - Record/replay: log raw streams to file and replay them through candidate mapping algorithms offline — this is also how the Phase 3 dipole model gets prototyped before porting to the RP2040.
- **Location**: `tools/tuner/` in this repo, with its own CMakeLists and a README.

### Suggested build order

1. Firmware serial protocol + settings-in-flash refactor (needed by everything else).
2. App scaffold: serial connect, live plots, 6DoF preview, tuning sliders.
3. Crosstalk metric panel (baseline measurement of the current firmware).
4. Calibration wizard + matrix upload (lands together with firmware Phase 2).
5. Record/replay for offline algorithm work (feeds Phase 3).

---

## Summary of concrete next steps

| # | Item | Area | Effort | Impact |
|---|------|------|--------|--------|
| 1 | Soft dead band + filter-before-deadband | Firmware (`MotionController`) | Small | Removes output snapping |
| 2 | Baseline auto-rezero when idle | Firmware | Small | Masks drift and spring sag |
| 3 | Guided 12-pose calibration → pseudo-inverse decoupling matrix, persisted to flash | Firmware | Medium | Fixes axis bleed |
| 4 | Travel limiters in knob/stem | Enclosure CAD | Small | Biggest spring-life win |
| 5 | Spring material & print guidance in README | Docs | Small | Immediate builder value |
| 6 | Root fillets / more arms in parametric spring | Enclosure CAD | Medium | Lower per-cycle stress |
| 7 | Serial command protocol + settings in flash (`Config` → runtime settings) | Firmware | Medium | Enables live tuning, no reflash |
| 8 | ImGui tuner app: plots, 6DoF preview, tuning sliders, crosstalk metric | Host app (`tools/tuner/`) | Medium | Preview + before/after measurement |
| 9 | Calibration wizard in app + matrix upload | Host app + firmware | Medium | Makes item 3 usable |
| 10 | Dipole-model pose solver (offline prototype via app record/replay) | Firmware/R&D | Large | Full nonlinearity fix |
| 11 | Knob revision for steel spring / O-rings | Enclosure CAD | Large | Fallback if 4–6 insufficient |

Items 1–2 and 5 are quick wins; item 3 is the core fix for the motion-processing complaint; items 4 and 6 address the spring without changing the build much; items 7–9 build the tuning/preview toolchain; items 10–11 are the fallbacks the author already anticipated.
