#include "Theme.h"

#include <cstdio>

#include "implot.h"

namespace theme {

void Apply() {
  ImGuiStyle& s = ImGui::GetStyle();

  // Geometry: generous padding, soft corners, pill grabs.
  s.WindowPadding = ImVec2(14, 12);
  s.FramePadding = ImVec2(10, 6);
  s.CellPadding = ImVec2(8, 5);
  s.ItemSpacing = ImVec2(10, 8);
  s.ItemInnerSpacing = ImVec2(8, 6);
  s.ScrollbarSize = 12.0f;
  s.GrabMinSize = 12.0f;

  s.WindowRounding = 10.0f;
  s.ChildRounding = 8.0f;
  s.FrameRounding = 6.0f;
  s.PopupRounding = 8.0f;
  s.ScrollbarRounding = 12.0f;
  s.GrabRounding = 12.0f;
  s.TabRounding = 6.0f;

  s.WindowBorderSize = 1.0f;
  s.ChildBorderSize = 1.0f;
  s.FrameBorderSize = 0.0f;
  s.PopupBorderSize = 1.0f;
  s.WindowTitleAlign = ImVec2(0.5f, 0.5f);
  s.SeparatorTextBorderSize = 1.0f;

  s.AntiAliasedLines = true;
  s.AntiAliasedFill = true;

  ImVec4* c = s.Colors;
  c[ImGuiCol_WindowBg] = kPage;
  c[ImGuiCol_ChildBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_PopupBg] = kSurfaceHi;
  c[ImGuiCol_Border] = kBorder;
  c[ImGuiCol_BorderShadow] = ImVec4(0, 0, 0, 0);

  c[ImGuiCol_Text] = kInk;
  c[ImGuiCol_TextDisabled] = kInkMuted;

  c[ImGuiCol_FrameBg] = kField;
  c[ImGuiCol_FrameBgHovered] = kFieldHover;
  c[ImGuiCol_FrameBgActive] = kFieldActive;

  c[ImGuiCol_TitleBg] = kSurface;
  c[ImGuiCol_TitleBgActive] = kSurfaceHi;
  c[ImGuiCol_TitleBgCollapsed] = kSurface;
  c[ImGuiCol_MenuBarBg] = kSurface;

  c[ImGuiCol_ScrollbarBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_ScrollbarGrab] = kField;
  c[ImGuiCol_ScrollbarGrabHovered] = kFieldHover;
  c[ImGuiCol_ScrollbarGrabActive] = kFieldActive;

  c[ImGuiCol_CheckMark] = kAccentHi;
  c[ImGuiCol_SliderGrab] = kAccent;
  c[ImGuiCol_SliderGrabActive] = kAccentHi;

  c[ImGuiCol_Button] = kField;
  c[ImGuiCol_ButtonHovered] = kFieldHover;
  c[ImGuiCol_ButtonActive] = kAccentSoft;

  c[ImGuiCol_Header] = kAccentSoft;
  c[ImGuiCol_HeaderHovered] = kFieldHover;
  c[ImGuiCol_HeaderActive] = kFieldActive;

  c[ImGuiCol_Separator] = kBorder;
  c[ImGuiCol_SeparatorHovered] = kAccentSoft;
  c[ImGuiCol_SeparatorActive] = kAccent;

  c[ImGuiCol_ResizeGrip] = ImVec4(1, 1, 1, 0.05f);
  c[ImGuiCol_ResizeGripHovered] = kAccentSoft;
  c[ImGuiCol_ResizeGripActive] = kAccent;

  c[ImGuiCol_Tab] = kSurface;
  c[ImGuiCol_TabHovered] = kFieldHover;
  c[ImGuiCol_TabActive] = kSurfaceHi;
  c[ImGuiCol_TabUnfocused] = kSurface;
  c[ImGuiCol_TabUnfocusedActive] = kSurfaceHi;

  c[ImGuiCol_TableHeaderBg] = kSurfaceHi;
  c[ImGuiCol_TableBorderStrong] = kBorder;
  c[ImGuiCol_TableBorderLight] = ImVec4(1, 1, 1, 0.04f);
  c[ImGuiCol_TableRowBg] = ImVec4(0, 0, 0, 0);
  c[ImGuiCol_TableRowBgAlt] = ImVec4(1, 1, 1, 0.02f);

  c[ImGuiCol_TextSelectedBg] = kAccentSoft;
  c[ImGuiCol_DragDropTarget] = kAccentHi;
  c[ImGuiCol_NavHighlight] = kAccentHi;
  c[ImGuiCol_ModalWindowDimBg] = ImVec4(0, 0, 0, 0.5f);

  // ImPlot: recessive chrome, the data carries the color.
  ImPlotStyle& p = ImPlot::GetStyle();
  p.LineWeight = 2.0f;
  p.PlotPadding = ImVec2(10, 10);
  p.LegendPadding = ImVec2(8, 8);
  p.LegendInnerPadding = ImVec2(8, 6);

  ImVec4* pc = p.Colors;
  pc[ImPlotCol_FrameBg] = ImVec4(0, 0, 0, 0);
  pc[ImPlotCol_PlotBg] = kSurface;
  pc[ImPlotCol_PlotBorder] = kBorder;
  pc[ImPlotCol_LegendBg] = ImVec4(kSurfaceHi.x, kSurfaceHi.y, kSurfaceHi.z, 0.88f);
  pc[ImPlotCol_LegendBorder] = kBorder;
  pc[ImPlotCol_LegendText] = kInk;
  pc[ImPlotCol_TitleText] = kInk;
  pc[ImPlotCol_AxisText] = kInkMuted;
  pc[ImPlotCol_AxisGrid] = kGrid;
  pc[ImPlotCol_AxisTick] = kGrid;
  pc[ImPlotCol_Crosshairs] = ImVec4(kInkMuted.x, kInkMuted.y, kInkMuted.z, 0.5f);
}

void LoadFonts() {
  ImGuiIO& io = ImGui::GetIO();

  const char* candidates[] = {
      "C:\\Windows\\Fonts\\SegUIVar.ttf",   // Segoe UI Variable (Win 11)
      "C:\\Windows\\Fonts\\segoeui.ttf",    // Segoe UI (Win 10)
  };
  for (const char* path : candidates) {
    FILE* f = std::fopen(path, "rb");
    if (f == nullptr) continue;
    std::fclose(f);
    if (io.Fonts->AddFontFromFileTTF(path, 17.0f) != nullptr) {
      return;
    }
  }
  io.Fonts->AddFontDefault();
}

}  // namespace theme
