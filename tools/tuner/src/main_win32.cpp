// Win32 + DirectX 11 bootstrap for the tuner, based on the stock Dear ImGui
// example (examples/example_win32_directx11).
#include <d3d11.h>
#include <tchar.h>

#include "App.h"
#include "imgui.h"
#include "backends/imgui_impl_dx11.h"
#include "backends/imgui_impl_win32.h"
#include "implot.h"

static ID3D11Device* g_pd3dDevice = nullptr;
static ID3D11DeviceContext* g_pd3dDeviceContext = nullptr;
static IDXGISwapChain* g_pSwapChain = nullptr;
static ID3D11RenderTargetView* g_mainRenderTargetView = nullptr;

static void CreateRenderTarget() {
  ID3D11Texture2D* pBackBuffer = nullptr;
  g_pSwapChain->GetBuffer(0, IID_PPV_ARGS(&pBackBuffer));
  if (pBackBuffer != nullptr) {
    g_pd3dDevice->CreateRenderTargetView(pBackBuffer, nullptr,
                                         &g_mainRenderTargetView);
    pBackBuffer->Release();
  }
}

static void CleanupRenderTarget() {
  if (g_mainRenderTargetView != nullptr) {
    g_mainRenderTargetView->Release();
    g_mainRenderTargetView = nullptr;
  }
}

static bool CreateDeviceD3D(HWND hWnd) {
  DXGI_SWAP_CHAIN_DESC sd = {};
  sd.BufferCount = 2;
  sd.BufferDesc.Width = 0;
  sd.BufferDesc.Height = 0;
  sd.BufferDesc.Format = DXGI_FORMAT_R8G8B8A8_UNORM;
  sd.BufferDesc.RefreshRate.Numerator = 60;
  sd.BufferDesc.RefreshRate.Denominator = 1;
  sd.Flags = DXGI_SWAP_CHAIN_FLAG_ALLOW_MODE_SWITCH;
  sd.BufferUsage = DXGI_USAGE_RENDER_TARGET_OUTPUT;
  sd.OutputWindow = hWnd;
  sd.SampleDesc.Count = 1;
  sd.SampleDesc.Quality = 0;
  sd.Windowed = TRUE;
  sd.SwapEffect = DXGI_SWAP_EFFECT_DISCARD;

  const D3D_FEATURE_LEVEL levels[] = {D3D_FEATURE_LEVEL_11_0,
                                      D3D_FEATURE_LEVEL_10_0};
  D3D_FEATURE_LEVEL level;
  HRESULT res = D3D11CreateDeviceAndSwapChain(
      nullptr, D3D_DRIVER_TYPE_HARDWARE, nullptr, 0, levels, 2,
      D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &level,
      &g_pd3dDeviceContext);
  if (res == DXGI_ERROR_UNSUPPORTED) {
    res = D3D11CreateDeviceAndSwapChain(
        nullptr, D3D_DRIVER_TYPE_WARP, nullptr, 0, levels, 2,
        D3D11_SDK_VERSION, &sd, &g_pSwapChain, &g_pd3dDevice, &level,
        &g_pd3dDeviceContext);
  }
  if (res != S_OK) return false;

  CreateRenderTarget();
  return true;
}

static void CleanupDeviceD3D() {
  CleanupRenderTarget();
  if (g_pSwapChain) {
    g_pSwapChain->Release();
    g_pSwapChain = nullptr;
  }
  if (g_pd3dDeviceContext) {
    g_pd3dDeviceContext->Release();
    g_pd3dDeviceContext = nullptr;
  }
  if (g_pd3dDevice) {
    g_pd3dDevice->Release();
    g_pd3dDevice = nullptr;
  }
}

extern IMGUI_IMPL_API LRESULT ImGui_ImplWin32_WndProcHandler(
    HWND hWnd, UINT msg, WPARAM wParam, LPARAM lParam);

static LRESULT WINAPI WndProc(HWND hWnd, UINT msg, WPARAM wParam,
                              LPARAM lParam) {
  if (ImGui_ImplWin32_WndProcHandler(hWnd, msg, wParam, lParam)) return true;

  switch (msg) {
    case WM_SIZE:
      if (g_pd3dDevice != nullptr && wParam != SIZE_MINIMIZED) {
        CleanupRenderTarget();
        g_pSwapChain->ResizeBuffers(0, LOWORD(lParam), HIWORD(lParam),
                                    DXGI_FORMAT_UNKNOWN, 0);
        CreateRenderTarget();
      }
      return 0;
    case WM_SYSCOMMAND:
      if ((wParam & 0xfff0) == SC_KEYMENU) return 0;  // disable ALT menu
      break;
    case WM_DESTROY:
      PostQuitMessage(0);
      return 0;
  }
  return DefWindowProc(hWnd, msg, wParam, lParam);
}

int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int) {
  WNDCLASSEXW wc = {sizeof(wc),
                    CS_CLASSDC,
                    WndProc,
                    0L,
                    0L,
                    GetModuleHandle(nullptr),
                    nullptr,
                    nullptr,
                    nullptr,
                    nullptr,
                    L"CADMouseTuner",
                    nullptr};
  RegisterClassExW(&wc);
  HWND hwnd = CreateWindowW(wc.lpszClassName, L"CAD Mouse MK2 Tuner",
                            WS_OVERLAPPEDWINDOW, 60, 60, 1380, 920, nullptr,
                            nullptr, wc.hInstance, nullptr);

  if (!CreateDeviceD3D(hwnd)) {
    CleanupDeviceD3D();
    UnregisterClassW(wc.lpszClassName, wc.hInstance);
    return 1;
  }

  ShowWindow(hwnd, SW_SHOWDEFAULT);
  UpdateWindow(hwnd);

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext();
  ImGuiIO& io = ImGui::GetIO();
  io.IniFilename = "cadmouse_tuner.ini";
  ImGui::StyleColorsDark();

  ImGui_ImplWin32_Init(hwnd);
  ImGui_ImplDX11_Init(g_pd3dDevice, g_pd3dDeviceContext);

  {
    App app;
    bool done = false;
    while (!done) {
      MSG msg;
      while (PeekMessage(&msg, nullptr, 0U, 0U, PM_REMOVE)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
        if (msg.message == WM_QUIT) done = true;
      }
      if (done) break;

      ImGui_ImplDX11_NewFrame();
      ImGui_ImplWin32_NewFrame();
      ImGui::NewFrame();

      app.frame();

      ImGui::Render();
      const float clear[4] = {0.06f, 0.07f, 0.09f, 1.0f};
      g_pd3dDeviceContext->OMSetRenderTargets(1, &g_mainRenderTargetView,
                                              nullptr);
      g_pd3dDeviceContext->ClearRenderTargetView(g_mainRenderTargetView,
                                                 clear);
      ImGui_ImplDX11_RenderDrawData(ImGui::GetDrawData());

      g_pSwapChain->Present(1, 0);  // vsync
    }
  }

  ImGui_ImplDX11_Shutdown();
  ImGui_ImplWin32_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

  CleanupDeviceD3D();
  DestroyWindow(hwnd);
  UnregisterClassW(wc.lpszClassName, wc.hInstance);
  return 0;
}
