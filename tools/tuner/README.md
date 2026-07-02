# CAD Mouse MK2 Tuner

A Windows desktop app (Dear ImGui + ImPlot) for previewing, calibrating, and
tuning the CAD Mouse MK2 live over USB serial — no reflashing needed.

![panels: Connection, Tuning, Signals, 6DoF Preview, Calibration Wizard, Crosstalk, Console]

## What it does

- **Live signals** — scrolling plots of all nine raw magnetometer channels and
  the six axis outputs.
- **6DoF preview** — bar meters plus a 3D cube driven by the live pose, so you
  can see exactly what the device is sending.
- **Tuning** — sliders for gains, dead zones, smoothing, response curve, and
  auto-re-zero, applied to the device instantly. `Save to flash` persists them
  across power cycles; `Export Config.h` writes a snippet if you prefer to
  bake values into the firmware source.
- **Calibration wizard** — the guided-sweep procedure that measures how *your*
  unit responds to each motion and computes a decoupling matrix that cancels
  axis bleed. No pure single-axis motions required: holds anchor each axis,
  slow circles fill in the rest, and the math separates the contamination.
- **Crosstalk panel** — drive one axis for 5 seconds and get a 6×6 heatmap of
  bleed percentages. Use it to measure before/after the calibration.
- **CSV logging** — record raw streams for offline algorithm work.

## Building

Requires Visual Studio 2022 (or Build Tools) with the C++ workload, CMake
3.20+, and internet access on first configure (imgui/implot are fetched
automatically).

```
cd tools/tuner
cmake -B build
cmake --build build --config Release
build\Release\cadmouse_tuner.exe
```

The math/protocol core has an off-target test suite that also runs on
Linux/macOS: `cmake -B build && cmake --build build && ./build/test_core`.

## Using it

1. Flash the updated firmware (this repo, `pio run -t upload`), plug the
   device in, launch the tuner, pick the COM port, **Connect**.
2. The app switches the device to full streaming and mirrors its settings.
3. Run **Zero** with hands off if the rest point looks offset.
4. Run the **Calibration Wizard** (~2 minutes, 7 steps). After **Apply**,
   check each axis in the preview, fix any inverted directions in
   *Tuning → Axis directions*, lower the dead zones to taste, then
   **Save to flash**.
5. Use the **Crosstalk** panel to verify the bleed actually dropped.

## Serial protocol (for scripting)

Line-based text over USB CDC at any baud. Commands: `PING`, `GET <name>`,
`GET *`, `SET <name> <value>`, `SAVE`, `LOAD`, `DEFAULTS`, `ZERO`,
`STREAM OFF|PLOT|OUT|FULL`, `MAT <row> <9 values>`, `MATON`, `MATOFF`,
`MAT?`. Stream lines: `O <ms> <6 outputs> <btn>` or
`D <ms> <9 raw> <3 temps> <6 outputs> <btn>`. See
[`src/Protocol.h`](src/Protocol.h) and the firmware's
[`CommandController`](../../firmware/include/controllers/CommandController.h).
