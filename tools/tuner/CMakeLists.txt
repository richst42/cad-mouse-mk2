cmake_minimum_required(VERSION 3.20)
project(cadmouse_tuner CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Core sources are platform independent and unit-testable off Windows.
set(CORE_SOURCES
    src/LinAlg.cpp
    src/Calibration.cpp
    src/Protocol.cpp)

if(WIN32)
  include(FetchContent)

  FetchContent_Declare(imgui
    URL https://github.com/ocornut/imgui/archive/refs/tags/v1.90.4.zip)
  FetchContent_Declare(implot
    URL https://github.com/epezent/implot/archive/refs/tags/v0.16.zip)
  FetchContent_MakeAvailable(imgui implot)

  add_executable(cadmouse_tuner WIN32
    ${CORE_SOURCES}
    src/SerialPort.cpp
    src/Flasher.cpp
    src/App.cpp
    src/main_win32.cpp
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_win32.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_dx11.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp)

  target_include_directories(cadmouse_tuner PRIVATE
    src
    ${imgui_SOURCE_DIR}
    ${implot_SOURCE_DIR})

  target_link_libraries(cadmouse_tuner PRIVATE d3d11 dxgi d3dcompiler comdlg32)
  target_compile_definitions(cadmouse_tuner PRIVATE UNICODE _UNICODE)
else()
  # Off-Windows we only build the math/protocol tests.
  add_executable(test_core tests/test_core.cpp ${CORE_SOURCES})
  target_include_directories(test_core PRIVATE src)
  enable_testing()
  add_test(NAME core COMMAND test_core)
endif()
