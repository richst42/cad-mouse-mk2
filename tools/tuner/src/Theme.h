#pragma once

#include "imgui.h"

// Design tokens and style application for the tuner's dark theme.
//
// Chart colors follow a validated dark-mode palette (categorical slots for
// the six axes; per-sensor hue ramps for the nine raw channels so sensor =
// hue and component = lightness). Status colors are reserved for the
// crosstalk severity tint and never reused as series colors.
namespace theme {

// --- surfaces & ink -------------------------------------------------------
const ImVec4 kPage{0.075f, 0.075f, 0.094f, 1.0f};        // #131318
const ImVec4 kSurface{0.090f, 0.090f, 0.113f, 1.0f};     // #17171d
const ImVec4 kSurfaceHi{0.106f, 0.106f, 0.133f, 1.0f};   // #1b1b22
const ImVec4 kField{0.133f, 0.133f, 0.169f, 1.0f};       // #22222b
const ImVec4 kFieldHover{0.165f, 0.165f, 0.208f, 1.0f};  // #2a2a35
const ImVec4 kFieldActive{0.196f, 0.196f, 0.247f, 1.0f}; // #32323f
const ImVec4 kInk{0.925f, 0.925f, 0.945f, 1.0f};         // primary text
const ImVec4 kInkMuted{0.545f, 0.545f, 0.588f, 1.0f};    // #8b8b96
const ImVec4 kGrid{0.173f, 0.173f, 0.200f, 1.0f};        // #2c2c33
const ImVec4 kBorder{1.0f, 1.0f, 1.0f, 0.08f};

// --- accent (interactive UI, not data) -------------------------------------
const ImVec4 kAccent{0.224f, 0.529f, 0.898f, 1.0f};      // #3987e5
const ImVec4 kAccentHi{0.333f, 0.596f, 0.906f, 1.0f};    // #5598e7
const ImVec4 kAccentSoft{0.224f, 0.529f, 0.898f, 0.28f};

// --- data: six output axes (categorical slots 1-6, dark mode) --------------
// Tx #3987e5  Ty #199e70  Tz #c98500  Rx #008300  Ry #9085e9  Rz #e66767
const ImVec4 kAxisColors[6] = {
    {0.224f, 0.529f, 0.898f, 1.0f}, {0.098f, 0.620f, 0.439f, 1.0f},
    {0.788f, 0.522f, 0.000f, 1.0f}, {0.000f, 0.514f, 0.000f, 1.0f},
    {0.565f, 0.522f, 0.914f, 1.0f}, {0.902f, 0.404f, 0.404f, 1.0f}};

// --- data: nine raw channels — sensor = hue, component (x/y/z) = lightness
// blue ramp (mag1), aqua ramp (mag2), orange ramp (mag3)
const ImVec4 kRawColors[9] = {
    {0.620f, 0.773f, 0.957f, 1.0f},  // m1x #9ec5f4
    {0.333f, 0.596f, 0.906f, 1.0f},  // m1y #5598e7
    {0.165f, 0.471f, 0.839f, 1.0f},  // m1z #2a78d6
    {0.510f, 0.863f, 0.714f, 1.0f},  // m2x #82dcb6
    {0.169f, 0.749f, 0.529f, 1.0f},  // m2y #2bbf87
    {0.098f, 0.620f, 0.439f, 1.0f},  // m2z #199e70
    {0.949f, 0.675f, 0.467f, 1.0f},  // m3x #f2ac77
    {0.910f, 0.463f, 0.247f, 1.0f},  // m3y #e8763f
    {0.780f, 0.306f, 0.114f, 1.0f}   // m3z #c74e1d
};

// --- status (crosstalk severity; reserved, never a series color) -----------
const ImVec4 kGood{0.047f, 0.639f, 0.047f, 1.0f};      // #0ca30c
const ImVec4 kWarning{0.980f, 0.698f, 0.098f, 1.0f};   // #fab219
const ImVec4 kCritical{0.816f, 0.231f, 0.231f, 1.0f};  // #d03b3b

// Applies the ImGui style (colors, rounding, spacing) and the matching
// ImPlot style. Call once after the contexts exist.
void Apply();

// Loads a modern UI font from the Windows system font directory (Segoe UI
// Variable, falling back to Segoe UI, falling back to the ImGui default).
// Call before the first frame.
void LoadFonts();

}  // namespace theme
