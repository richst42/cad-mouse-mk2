You can customize several variables to tune gains, smoothing, and deadzones for all six axes.
Most of these settings are defined in [`Config.h`](include/Config.h) and are the boot defaults — they can now also be changed **live over USB serial** with the [desktop tuner app](../tools/tuner/) and saved to flash, so editing and reflashing is no longer required for tuning.

```cpp
// Gains and sign fixes
const float GAIN_T[3] = {28.0, 28.0, 24.0};
const float GAIN_R[3] = {18.0, 18.0, 20.0};
const int SIGN_AXIS[6] = {-1, +1, -1, +1, +1, +1};

// Dead zones
const float DEAD_T = 16.0;
const float DEAD_R = 20.0;

// Smoothing
const float SMOOTH_TAU_S = 0.08;

// Response curve exponent (1.0 = linear)
const float CURVE_EXP = 1.0;

// Slow baseline re-zero while at rest
const bool REZERO_ENABLED = true;
const float REZERO_DELAY_S = 2.0;
const float REZERO_TAU_S = 10.0;
```

⚠️ Refer to the video at [6:23](https://youtu.be/62xlzGs8LXA?si=ld2shDCaTxOLIGB8&t=383) for a demo of driver support. Related settings can be found commented in[`platformio.ini`](../platformio.ini).

## Motion processing

The pipeline in [`MotionController`](src/controllers/MotionController.cpp) now maps the nine baseline-subtracted sensor deltas to the six axes through a **6×9 matrix**:

- By default it uses a geometry-derived matrix equivalent to the original hand-derived formulas (translations = per-component averages, rotations = z-differences and the x/y swirl term for the triangle layout below).
- After running the **guided calibration** in the [tuner app](../tools/tuner/), a per-unit measured matrix replaces it. Because it is the pseudo-inverse of the unit's actual response directions, it cancels the inter-axis bleed that the fixed formulas could not.

Downstream of the matrix: per-axis gain and sign, a low-pass filter, then a **soft dead band** (output ramps smoothly from zero at the band edge instead of snapping) with an optional response-curve exponent. While the device sits still, the baseline is slowly **re-zeroed** to absorb thermal drift and spring settling.

Known remaining limitation: the calibrated matrix is a first-order (linear) model. The magnetic field is nonlinear with displacement, so some residual crosstalk remains at large deflections; a model-based pose solver could replace this eventually (see [docs/FIX_PLAN.md](../docs/FIX_PLAN.md)). Contributions welcome.

**Sensor layout:**
- `mag1` = bottom
- `mag2` = top left
- `mag3` = top right

## Serial interface

The USB serial port carries telemetry and a command channel (both handled by [`CommandController`](src/controllers/CommandController.cpp) / [`TelemetryController`](src/controllers/TelemetryController.cpp)):

| Command | Effect |
|---|---|
| `PING` | identify: `PONG cad-mouse-mk2 1` |
| `GET <name>` / `GET *` | read one/all runtime settings |
| `SET <name> <value>` | change a setting live (e.g. `SET dead_t 8`) |
| `SAVE` / `LOAD` / `DEFAULTS` | persist to / reload from flash, factory reset |
| `ZERO` | re-run the rest-pose zero calibration |
| `STREAM OFF\|PLOT\|OUT\|FULL` | telemetry mode (`PLOT` = Teleplot format, `FULL` = raw + temps + outputs) |
| `MAT <row> <9 values>`, `MATON`, `MATOFF`, `MAT?` | upload/enable/inspect the decoupling matrix |

Parameter names: `gain_tx|ty|tz|rx|ry|rz`, `sign_*`, `dead_t`, `dead_r`, `tau`, `curve`, `rezero_on`, `rezero_delay`, `rezero_tau`, `led_bright`, `led_idle`, `led_cal` (colors are decimal `0xRRGGBB` values).
