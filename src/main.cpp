// ============================================================================
// WiggleMe AI - Human-Kinematic Mouse Simulator with AI Movement Engine
// Merged: Proven WigglerLoop + AI Fingerprinting + Modern Dark UI
// ============================================================================

#ifndef UNICODE
#define UNICODE
#endif

#include <windows.h>
#include <commctrl.h>
#include <shellapi.h>
#include <tlhelp32.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iomanip>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "../include/ai_movement.h"

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "shlwapi.lib")

// ============================================================================
// Algorithm IDs
// ============================================================================
#define ALGO_WINDMOUSE 0
#define ALGO_ENHANCED_WM 1
#define ALGO_BEZIER 2
#define ALGO_FITTS 3
#define ALGO_AI_ENGINE 4

// ============================================================================
// UI IDs & Messages
// ============================================================================
#define WM_UPDATE_STATUS (WM_APP + 1)
#define WM_TRAYICON (WM_APP + 2)
#define IDI_TRAYICON 1001
#define IDC_START_BTN 2001
#define IDC_RECORD_BTN 2002
#define IDC_ALGO_COMBO 2003
#define IDC_CHAOS_CHECK 2004
#define IDC_ONTOP_CHECK 2005
#define IDC_PITCH_SLIDER 2006
#define IDC_YAW_SLIDER 2007
#define IDC_INT_SLIDER 2008
#define IDC_DELAY_SLIDER 2009

// ============================================================================
// Colors – THE MATRIX
// ============================================================================
#define CLR_BG RGB(0, 0, 0)
#define CLR_SURFACE RGB(2, 10, 2)
#define CLR_CARD RGB(5, 20, 5)
#define CLR_ACCENT RGB(0, 255, 65)
#define CLR_ACCENT_HI RGB(120, 255, 160)
#define CLR_GREEN RGB(0, 255, 65)
#define CLR_RED RGB(255, 40, 40)
#define CLR_YELLOW RGB(200, 255, 50)
#define CLR_CYAN RGB(0, 255, 180)
#define CLR_TEXT RGB(0, 255, 65)
#define CLR_TEXT_DIM RGB(0, 128, 32)
#define CLR_BORDER RGB(0, 80, 20)
#define CLR_GLOW RGB(0, 200, 50)
#define CLR_RAIN_HEAD RGB(220, 255, 220)
#define CLR_RAIN_BRIGHT RGB(0, 255, 65)
#define CLR_RAIN_DIM RGB(0, 100, 25)
#define CLR_RAIN_FAINT RGB(0, 40, 10)
#define CLR_SCANLINE RGB(0, 8, 0)

// ============================================================================
// Matrix Digital Rain
// ============================================================================
#define RAIN_COLS 40
#define RAIN_MAX_LEN 28
#define RAIN_CHAR_H 16
#define RAIN_CHAR_W 10

struct RainColumn {
  double y;                // current head position (pixels)
  double speed;            // fall speed (pixels per frame)
  int length;              // trail length in characters
  int chars[RAIN_MAX_LEN]; // character indices
  int x;                   // pixel X position
};

static RainColumn g_rain[RAIN_COLS];
static bool g_rainInit = false;
static HFONT g_hFontRain = NULL;
static int g_frameCount = 0;

static const wchar_t *MATRIX_CHARS =
    L"ｱｲｳｴｵｶｷｸｹｺｻｼｽｾｿﾀﾁﾂﾃﾅﾆﾇﾈﾉﾊﾋﾌﾍﾎﾏﾐﾑﾒﾓﾔﾕﾗﾘﾙﾚﾛﾜﾝ0123456789ABCDEFZ";
static int g_matrixCharCount = 0;

static void InitRain(int winW, int winH) {
  g_matrixCharCount = (int)wcslen(MATRIX_CHARS);
  for (int i = 0; i < RAIN_COLS; i++) {
    g_rain[i].x = (i * winW) / RAIN_COLS + rand() % 6;
    g_rain[i].y = -(double)(rand() % (winH + 200));
    g_rain[i].speed = 1.5 + (rand() % 30) / 10.0;
    g_rain[i].length = 5 + rand() % (RAIN_MAX_LEN - 5);
    for (int j = 0; j < RAIN_MAX_LEN; j++)
      g_rain[i].chars[j] = rand() % g_matrixCharCount;
  }
  g_rainInit = true;
}

static void UpdateAndDrawRain(HDC hdc, int winW, int winH) {
  if (!g_rainInit)
    InitRain(winW, winH);
  SelectObject(hdc, g_hFontRain);
  SetBkMode(hdc, TRANSPARENT);
  g_frameCount++;
  for (int i = 0; i < RAIN_COLS; i++) {
    RainColumn &col = g_rain[i];
    col.y += col.speed;
    if (col.y - col.length * RAIN_CHAR_H > winH) {
      col.y = -(double)(rand() % 300);
      col.speed = 1.5 + (rand() % 30) / 10.0;
      col.length = 5 + rand() % (RAIN_MAX_LEN - 5);
      for (int j = 0; j < RAIN_MAX_LEN; j++)
        col.chars[j] = rand() % g_matrixCharCount;
    }
    // Randomly mutate one character per frame
    if (rand() % 3 == 0)
      col.chars[rand() % col.length] = rand() % g_matrixCharCount;
    for (int j = 0; j < col.length; j++) {
      int cy = (int)col.y - j * RAIN_CHAR_H;
      if (cy < -RAIN_CHAR_H || cy > winH)
        continue;
      COLORREF clr;
      if (j == 0)
        clr = CLR_RAIN_HEAD;
      else if (j < 3)
        clr = CLR_RAIN_BRIGHT;
      else if (j < col.length / 2)
        clr = CLR_RAIN_DIM;
      else
        clr = CLR_RAIN_FAINT;
      SetTextColor(hdc, clr);
      wchar_t ch = MATRIX_CHARS[col.chars[j] % g_matrixCharCount];
      TextOutW(hdc, col.x, cy, &ch, 1);
    }
  }
}

static void DrawScanlines(HDC hdc, int w, int h) {
  HPEN pen = CreatePen(PS_SOLID, 1, CLR_SCANLINE);
  HPEN oldPen = (HPEN)SelectObject(hdc, pen);
  for (int y = 0; y < h; y += 3) {
    MoveToEx(hdc, 0, y, NULL);
    LineTo(hdc, w, y);
  }
  SelectObject(hdc, oldPen);
  DeleteObject(pen);
}

// ============================================================================
// Globals
// ============================================================================
static HWND g_hWnd = NULL;
static HWND g_hStartBtn, g_hRecordBtn;
static HWND g_hPitchSlider, g_hYawSlider, g_hIntervalSlider, g_hDelaySlider;
static HWND g_hPitchVal, g_hYawVal, g_hIntervalVal, g_hDelayVal;
static HWND g_hAlgoCombo, g_hChaosCheck, g_hTopCheck;
static HWND g_hStatus, g_hScoreLabel, g_hFPStatus;

static HFONT g_hFontTitle = NULL;
static HFONT g_hFontBig = NULL;
static HFONT g_hFontNormal = NULL;
static HFONT g_hFontSmall = NULL;
static HBRUSH g_hBrBg = NULL;
static HBRUSH g_hBrCard = NULL;

static NOTIFYICONDATA g_nid = {};
static bool g_bTrayActive = false;

// Engine state
static AIMovementEngine g_engine;
static std::atomic<bool> g_bRunning(false);
static std::atomic<bool> g_bRecording(false);
static std::atomic<int> g_pitch(50), g_yaw(50);
static std::atomic<int> g_delay(5);
static std::atomic<double> g_interval(5.0);
static std::atomic<int> g_algorithm(ALGO_WINDMOUSE);
static std::atomic<bool> g_bChaosMode(false);
static std::atomic<int> g_overrideThreshold(30);
static std::atomic<double> g_lastScore(0.0);
static std::atomic<int> g_movementCount(0);

// ============================================================================
// Perlin Noise (1D)
// ============================================================================
static int perm[512];
static bool permInitialized = false;

static void InitPerlin() {
  if (permInitialized)
    return;
  int p[256];
  for (int i = 0; i < 256; i++)
    p[i] = i;
  for (int i = 255; i > 0; i--) {
    int j = rand() % (i + 1);
    int tmp = p[i];
    p[i] = p[j];
    p[j] = tmp;
  }
  for (int i = 0; i < 512; i++)
    perm[i] = p[i & 255];
  permInitialized = true;
}

static double Fade(double t) { return t * t * t * (t * (t * 6 - 15) + 10); }
static double Lerp(double a, double b, double t) { return a + t * (b - a); }
static double Grad1D(int hash, double x) { return (hash & 1) ? x : -x; }

static double Perlin1D(double x) {
  InitPerlin();
  int xi = (int)floor(x) & 255;
  double xf = x - floor(x);
  double u = Fade(xf);
  return Lerp(Grad1D(perm[xi], xf), Grad1D(perm[xi + 1], xf - 1.0), u);
}

// ============================================================================
// DPI helper
// ============================================================================
static int GetDpiScale(HWND hwnd) {
  UINT dpi = 96;
  HMODULE hUser32 = GetModuleHandle(L"user32.dll");
  if (hUser32) {
    typedef UINT(WINAPI * GetDpiForWindowFunc)(HWND);
    auto pGetDpi =
        (GetDpiForWindowFunc)GetProcAddress(hUser32, "GetDpiForWindow");
    if (pGetDpi)
      dpi = pGetDpi(hwnd);
  }
  return MulDiv(30, dpi, 96);
}

// ============================================================================
// System Tray
// ============================================================================
static void AddTrayIcon(HWND hwnd) {
  g_nid.cbSize = sizeof(NOTIFYICONDATA);
  g_nid.hWnd = hwnd;
  g_nid.uID = IDI_TRAYICON;
  g_nid.uFlags = NIF_ICON | NIF_MESSAGE | NIF_TIP;
  g_nid.uCallbackMessage = WM_TRAYICON;
  g_nid.hIcon = LoadIcon(NULL, IDI_APPLICATION);
  wcscpy_s(g_nid.szTip, _countof(g_nid.szTip), L"WiggleMe AI");
  Shell_NotifyIcon(NIM_ADD, &g_nid);
  g_bTrayActive = true;
}

static void RemoveTrayIcon() {
  if (g_bTrayActive) {
    Shell_NotifyIcon(NIM_DELETE, &g_nid);
    g_bTrayActive = false;
  }
}

// ============================================================================
// Algorithm 1: WindMouse (Classic)
// ============================================================================
static void WindMouse(double startX, double startY, double endX, double endY,
                      double gravity, double wind, double minWait,
                      double maxWait, double maxStep, double targetArea) {
  double dist = hypot(endX - startX, endY - startY);
  double windX = 0, windY = 0, vx = 0, vy = 0;
  double cx = startX, cy = startY;

  while (dist > 1.0 && g_bRunning) {
    wind = (double)(std::min)((int)wind, (int)dist);
    if (dist >= targetArea) {
      windX = windX / sqrt(3) +
              (rand() % (std::max)(1, (int)(wind * 2 + 1)) - wind) / sqrt(5);
      windY = windY / sqrt(3) +
              (rand() % (std::max)(1, (int)(wind * 2 + 1)) - wind) / sqrt(5);
    } else {
      windX /= 2;
      windY /= 2;
      if (maxStep < 3)
        maxStep = (double)(rand() % 3 + 3);
      else
        maxStep /= 1.5;
    }

    vx += windX + gravity * (endX - cx) / dist;
    vy += windY + gravity * (endY - cy) / dist;

    double vmag = hypot(vx, vy);
    if (vmag > maxStep) {
      int halfStep = (std::max)(1, (int)(maxStep / 2.0 + 1));
      double scale = (maxStep / 2.0 + (rand() % halfStep)) / vmag;
      vx *= scale;
      vy *= scale;
    }

    cx += vx;
    cy += vy;
    SetCursorPos((int)cx, (int)cy);

    dist = hypot(endX - cx, endY - cy);
    double wait =
        minWait + (rand() % (std::max)(1, (int)(maxWait - minWait + 1)));
    Sleep((DWORD)wait);
  }
}

// ============================================================================
// Algorithm 2: Enhanced WindMouse (Adaptive Speed + Perlin Jitter + Pauses)
// ============================================================================
static void EnhancedWindMouse(double startX, double startY, double endX,
                              double endY) {
  double totalDist = hypot(endX - startX, endY - startY);
  if (totalDist < 1.0)
    return;

  double gravity = 9.0, windMag = 3.0;
  double maxStep = 15.0, targetArea = (std::max)(12.0, totalDist * 0.15);
  double windX = 0, windY = 0, vx = 0, vy = 0;
  double cx = startX, cy = startY;
  double noiseT = (double)(rand() % 1000);

  while (g_bRunning) {
    double dist = hypot(endX - cx, endY - cy);
    if (dist <= 1.0)
      break;

    double speedFactor = (std::min)(1.0, dist / totalDist + 0.2);
    double curMaxStep = maxStep * speedFactor;

    double w = (std::min)(windMag, dist);
    if (dist >= targetArea) {
      windX = windX / sqrt(3) +
              (rand() % (std::max)(1, (int)(w * 2 + 1)) - w) / sqrt(5);
      windY = windY / sqrt(3) +
              (rand() % (std::max)(1, (int)(w * 2 + 1)) - w) / sqrt(5);
    } else {
      windX /= 2;
      windY /= 2;
      if (curMaxStep < 3)
        curMaxStep = (double)(rand() % 3 + 3);
      else
        curMaxStep /= 1.5;
    }

    vx += windX + gravity * (endX - cx) / dist;
    vy += windY + gravity * (endY - cy) / dist;

    double vmag = hypot(vx, vy);
    if (vmag > curMaxStep) {
      int halfStep = (std::max)(1, (int)(curMaxStep / 2.0 + 1));
      double scale = (curMaxStep / 2.0 + (rand() % halfStep)) / vmag;
      vx *= scale;
      vy *= scale;
    }

    noiseT += 0.05;
    double jitterX = Perlin1D(noiseT) * 2.5;
    double jitterY = Perlin1D(noiseT + 100.0) * 2.5;

    cx += vx + jitterX;
    cy += vy + jitterY;
    SetCursorPos((int)cx, (int)cy);

    if (rand() % 100 < 3) {
      Sleep((DWORD)(50 + rand() % 150));
    }

    double baseWait = 2.0 + (1.0 - speedFactor) * 8.0;
    Sleep((DWORD)(baseWait + rand() % 4));
  }
}

// ============================================================================
// Algorithm 3: Bezier Curve Path
// ============================================================================
struct Vec2 {
  double x, y;
};

static Vec2 CubicBezier(Vec2 p0, Vec2 p1, Vec2 p2, Vec2 p3, double t) {
  double u = 1 - t, uu = u * u, uuu = uu * u;
  double tt = t * t, ttt = tt * t;
  Vec2 p;
  p.x = uuu * p0.x + 3 * uu * t * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
  p.y = uuu * p0.y + 3 * uu * t * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
  return p;
}

static void BezierMove(double startX, double startY, double endX, double endY) {
  double dist = hypot(endX - startX, endY - startY);
  if (dist < 1.0)
    return;

  Vec2 p0 = {startX, startY};
  Vec2 p3 = {endX, endY};

  double perpX = -(endY - startY) / dist;
  double perpY = (endX - startX) / dist;

  double arc1 =
      dist * (0.1 + (rand() % 30) / 100.0) * ((rand() % 2) ? 1.0 : -1.0);
  double arc2 =
      dist * (0.1 + (rand() % 30) / 100.0) * ((rand() % 2) ? 1.0 : -1.0);

  Vec2 p1 = {startX + (endX - startX) * 0.3 + perpX * arc1,
             startY + (endY - startY) * 0.3 + perpY * arc1};
  Vec2 p2 = {startX + (endX - startX) * 0.7 + perpX * arc2,
             startY + (endY - startY) * 0.7 + perpY * arc2};

  int steps = (std::max)(10, (int)(dist / 2.0));
  steps = (std::min)(steps, 300);

  for (int i = 1; i <= steps && g_bRunning; i++) {
    double rawT = (double)i / steps;
    double t = rawT * rawT * (3.0 - 2.0 * rawT);
    Vec2 pos = CubicBezier(p0, p1, p2, p3, t);
    pos.x += Perlin1D(rawT * 10.0 + (double)(rand() % 100)) * 1.5;
    pos.y += Perlin1D(rawT * 10.0 + 50.0) * 1.5;
    SetCursorPos((int)pos.x, (int)pos.y);
    double speed = 1.0 + 4.0 * sin(rawT * 3.14159);
    double wait = (std::max)(1.0, 8.0 / speed);
    Sleep((DWORD)wait);
  }
}

// ============================================================================
// Algorithm 4: Fitts's Law + Overshoot
// ============================================================================
static void FittsMove(double startX, double startY, double endX, double endY) {
  double dist = hypot(endX - startX, endY - startY);
  if (dist < 1.0)
    return;

  double targetWidth = 20.0;
  double fittsTime = 100.0 + 150.0 * log2(2.0 * dist / targetWidth + 1.0);
  fittsTime = (std::max)(100.0, (std::min)(fittsTime, 2000.0));

  bool overshoot = (dist > 100.0) && (rand() % 100 < 60);
  double overshootDist = 0;
  if (overshoot)
    overshootDist = dist * (0.05 + (rand() % 15) / 100.0);

  double dirX = (endX - startX) / dist;
  double dirY = (endY - startY) / dist;
  double moveToX = overshoot ? endX + dirX * overshootDist : endX;
  double moveToY = overshoot ? endY + dirY * overshootDist : endY;
  double moveDist = hypot(moveToX - startX, moveToY - startY);

  int steps = (std::max)(10, (int)(fittsTime / 8.0));
  steps = (std::min)(steps, 300);

  double cx = startX, cy = startY;
  for (int i = 1; i <= steps && g_bRunning; i++) {
    double rawT = (double)i / steps;
    double t = 1.0 - pow(1.0 - rawT, 2.5);

    double targetX = startX + (moveToX - startX) * t;
    double targetY = startY + (moveToY - startY) * t;

    double perpX = -(moveToY - startY) / moveDist;
    double perpY = (moveToX - startX) / moveDist;
    double drift = Perlin1D(rawT * 8.0) * (dist * 0.03);
    targetX += perpX * drift;
    targetY += perpY * drift;
    targetX += Perlin1D(rawT * 20.0 + 200.0) * 1.2;
    targetY += Perlin1D(rawT * 20.0 + 300.0) * 1.2;

    cx = targetX;
    cy = targetY;
    SetCursorPos((int)cx, (int)cy);
    double wait = fittsTime / steps;
    Sleep((DWORD)(std::max)(1.0, wait));
  }

  if (overshoot && g_bRunning) {
    Sleep((DWORD)(30 + rand() % 70));
    double corrDist = hypot(endX - cx, endY - cy);
    int corrSteps = (std::max)(5, (int)(corrDist / 1.5));
    corrSteps = (std::min)(corrSteps, 80);
    double corrStartX = cx, corrStartY = cy;
    for (int i = 1; i <= corrSteps && g_bRunning; i++) {
      double rawT = (double)i / corrSteps;
      double t = rawT * rawT * (3.0 - 2.0 * rawT);
      double tx = corrStartX + (endX - corrStartX) * t;
      double ty = corrStartY + (endY - corrStartY) * t;
      tx += Perlin1D(rawT * 15.0 + 400.0) * 0.6;
      ty += Perlin1D(rawT * 15.0 + 500.0) * 0.6;
      SetCursorPos((int)tx, (int)ty);
      Sleep((DWORD)(4 + rand() % 6));
    }
  }

  if (g_bRunning) {
    SetCursorPos((int)endX + (rand() % 3 - 1), (int)endY + (rand() % 3 - 1));
  }
}

// ============================================================================
// Algorithm 5: AI Engine (uses ai_movement.cpp trajectory)
// ============================================================================
static void AIEngineMove(double startX, double startY, double endX,
                         double endY) {
  auto trajectory =
      g_engine.generateTrajectory(Point2D(startX, startY), Point2D(endX, endY));

  double score = g_engine.calculateAuthenticityScore(trajectory);
  g_lastScore.store(score);

  for (size_t i = 0; i < trajectory.size() && g_bRunning; i++) {
    SetCursorPos((int)trajectory[i].x, (int)trajectory[i].y);
    double baseDelay = 2.0;
    if (i > 0) {
      double d = trajectory[i].distTo(trajectory[i - 1]);
      baseDelay = (std::max)(1.0, d * 0.8);
    }
    Sleep((DWORD)baseDelay);
  }
  g_movementCount++;
}

// ============================================================================
// Dispatch: move cursor using selected algorithm
// ============================================================================
static void MoveWithAlgorithm(int algo, double sx, double sy, double ex,
                              double ey) {
  switch (algo) {
  case ALGO_WINDMOUSE:
    WindMouse(sx, sy, ex, ey, 9.0, 3.0, 2.0, 10.0, 10.0, 8.0);
    break;
  case ALGO_ENHANCED_WM:
    EnhancedWindMouse(sx, sy, ex, ey);
    break;
  case ALGO_BEZIER:
    BezierMove(sx, sy, ex, ey);
    break;
  case ALGO_FITTS:
    FittsMove(sx, sy, ex, ey);
    break;
  case ALGO_AI_ENGINE:
    AIEngineMove(sx, sy, ex, ey);
    break;
  default:
    WindMouse(sx, sy, ex, ey, 9.0, 3.0, 2.0, 10.0, 10.0, 8.0);
    break;
  }
}

// ============================================================================
// Wiggler Thread (proven loop from original WiggleMe)
// ============================================================================
static void WigglerLoop(HWND hMain) {
  int screenW = GetSystemMetrics(SM_CXSCREEN);
  int screenH = GetSystemMetrics(SM_CYSCREEN);

  POINT lastSetPos;
  GetCursorPos(&lastSetPos);
  auto startTime = std::chrono::steady_clock::now();
  int threshold = g_overrideThreshold.load();

  while (g_bRunning) {
    auto now = std::chrono::steady_clock::now();
    double elapsed =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime)
            .count() /
        1000.0;

    int p = g_pitch;
    int y = g_yaw;
    double interval = g_interval;
    int algo = g_algorithm;

    POINT cur;
    GetCursorPos(&cur);

    if (elapsed < (double)g_delay) {
      wchar_t *graceMsg = new wchar_t[100];
      swprintf(graceMsg, 100, L"Grace Period: %.1fs remaining...",
               (double)g_delay - elapsed);
      PostMessage(hMain, WM_UPDATE_STATUS, 0, (LPARAM)graceMsg);
      lastSetPos = cur;
    } else {
      if (abs(cur.x - lastSetPos.x) > threshold ||
          abs(cur.y - lastSetPos.y) > threshold) {
        g_bRunning = false;
        // Send safety stop
        PostMessage(hMain, WM_COMMAND, MAKEWPARAM(9999, 0), 0);
        break;
      }
      wchar_t *statusMsg = new wchar_t[80];
      wcscpy_s(statusMsg, 80, L"Wiggling... Move mouse to stop.");
      PostMessage(hMain, WM_UPDATE_STATUS, 0, (LPARAM)statusMsg);
    }

    if (g_bRunning && elapsed >= (double)g_delay) {
      if (g_bChaosMode) {
        int targetX = rand() % screenW;
        int targetY = rand() % screenH;
        MoveWithAlgorithm(algo, (double)cur.x, (double)cur.y, (double)targetX,
                          (double)targetY);
        GetCursorPos(&lastSetPos);
      } else if (p > 0 || y > 0) {
        int dx = (y > 0) ? (rand() % (y * 2 + 1)) - y : 0;
        int dy = (p > 0) ? (rand() % (p * 2 + 1)) - p : 0;
        MoveWithAlgorithm(algo, (double)cur.x, (double)cur.y,
                          (double)(cur.x + dx), (double)(cur.y + dy));
        GetCursorPos(&lastSetPos);
      }
    }

    ULONGLONG stop = GetTickCount64() + (ULONGLONG)(interval * 1000);
    while (GetTickCount64() < stop && g_bRunning) {
      GetCursorPos(&cur);
      auto nowCheck = std::chrono::steady_clock::now();
      double elap = std::chrono::duration_cast<std::chrono::milliseconds>(
                        nowCheck - startTime)
                        .count() /
                    1000.0;

      if (elap < (double)g_delay) {
        lastSetPos = cur;
      } else {
        if (abs(cur.x - lastSetPos.x) > threshold ||
            abs(cur.y - lastSetPos.y) > threshold) {
          g_bRunning = false;
          PostMessage(hMain, WM_COMMAND, MAKEWPARAM(9999, 0), 0);
          break;
        }
      }
      Sleep(100);
    }
  }
}

// ============================================================================
// Recording Thread - Fingerprint Learning
// ============================================================================
static std::thread g_recordThread;

static void RecordThread() {
  g_engine.startRecording();
  auto startTime = std::chrono::steady_clock::now();

  while (g_bRecording.load()) {
    POINT pos;
    GetCursorPos(&pos);
    bool clicking = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
    g_engine.addSample(Point2D((double)pos.x, (double)pos.y), clicking);
    Sleep(16); // ~60 samples/second
  }

  g_engine.stopRecording();
  g_engine.buildFingerprint();

  if (g_engine.hasFingerprint()) {
    wchar_t *msg = new wchar_t[128];
    auto &fp = g_engine.getFingerprint();
    swprintf(msg, 128, L"Fingerprint learned! Avg %.0f px/s", fp.avgSpeed);
    PostMessage(g_hWnd, WM_UPDATE_STATUS, 0, (LPARAM)msg);
  } else {
    wchar_t *msg = new wchar_t[80];
    wcscpy_s(msg, 80, L"Not enough data — record longer (2+ min)");
    PostMessage(g_hWnd, WM_UPDATE_STATUS, 0, (LPARAM)msg);
  }
}

// ============================================================================
// Drawing Helpers
// ============================================================================
static void FillRoundRect(HDC hdc, int x, int y, int w, int h, int r,
                          COLORREF fill, COLORREF border) {
  HBRUSH brush = CreateSolidBrush(fill);
  HPEN pen = CreatePen(PS_SOLID, 1, border);
  HBRUSH oldBrush = (HBRUSH)SelectObject(hdc, brush);
  HPEN oldPen = (HPEN)SelectObject(hdc, pen);
  RoundRect(hdc, x, y, x + w, y + h, r, r);
  SelectObject(hdc, oldBrush);
  SelectObject(hdc, oldPen);
  DeleteObject(brush);
  DeleteObject(pen);
}

static void DrawLabel(HDC hdc, const wchar_t *text, int x, int y,
                      COLORREF color, HFONT font) {
  SetTextColor(hdc, color);
  SetBkMode(hdc, TRANSPARENT);
  HFONT oldFont = (HFONT)SelectObject(hdc, font);
  TextOutW(hdc, x, y, text, (int)wcslen(text));
  SelectObject(hdc, oldFont);
}

static void DrawScoreGauge(HDC hdc, int cx, int cy, int radius, double score) {
  // Background circle
  HBRUSH bgBr = CreateSolidBrush(CLR_CARD);
  HPEN bgPen = CreatePen(PS_SOLID, 2, CLR_BORDER);
  SelectObject(hdc, bgBr);
  SelectObject(hdc, bgPen);
  Ellipse(hdc, cx - radius, cy - radius, cx + radius, cy + radius);
  DeleteObject(bgBr);
  DeleteObject(bgPen);

  // Score arc
  COLORREF arcColor =
      score >= 80 ? CLR_GREEN : (score >= 60 ? CLR_YELLOW : CLR_RED);
  HPEN arcPen = CreatePen(PS_SOLID, 5, arcColor);
  SelectObject(hdc, arcPen);
  SelectObject(hdc, GetStockObject(NULL_BRUSH));

  double startAngle = 3.14159 * 0.75;
  double endAngle = startAngle + (score / 100.0) * 3.14159 * 1.5;
  int segments = (std::max)(1, (int)(score / 2));
  for (int i = 0; i < segments; i++) {
    double a1 = startAngle + (double)i / segments * (endAngle - startAngle);
    double a2 =
        startAngle + (double)(i + 1) / segments * (endAngle - startAngle);
    MoveToEx(hdc, cx + (int)(cos(a1) * (radius - 4)),
             cy - (int)(sin(a1) * (radius - 4)), NULL);
    LineTo(hdc, cx + (int)(cos(a2) * (radius - 4)),
           cy - (int)(sin(a2) * (radius - 4)));
  }
  DeleteObject(arcPen);

  // Score number
  wchar_t buf[16];
  swprintf(buf, 16, L"%d", (int)score);
  SetTextColor(hdc, CLR_TEXT);
  SetBkMode(hdc, TRANSPARENT);
  SelectObject(hdc, g_hFontBig);
  SIZE sz;
  GetTextExtentPoint32W(hdc, buf, (int)wcslen(buf), &sz);
  TextOutW(hdc, cx - sz.cx / 2, cy - sz.cy / 2 - 4, buf, (int)wcslen(buf));

  // "/ 100" label
  SelectObject(hdc, g_hFontSmall);
  SetTextColor(hdc, CLR_TEXT_DIM);
  TextOutW(hdc, cx - 12, cy + 10, L"/ 100", 5);
}

// ============================================================================
// Slider Label Update
// ============================================================================
static void UpdateSliderLabels() {
  wchar_t buf[32];
  swprintf(buf, 32, L"%d px", g_pitch.load());
  SetWindowText(g_hPitchVal, buf);
  swprintf(buf, 32, L"%d px", g_yaw.load());
  SetWindowText(g_hYawVal, buf);
  swprintf(buf, 32, L"%d s", (int)g_interval.load());
  SetWindowText(g_hIntervalVal, buf);
  swprintf(buf, 32, L"%d s", g_delay.load());
  SetWindowText(g_hDelayVal, buf);
}

// ============================================================================
// Window Procedure
// ============================================================================
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam,
                                LPARAM lParam) {
  switch (msg) {
  case WM_CREATE: {
    g_hWnd = hwnd;

    // Fonts - Consolas for Matrix hacker aesthetic
    g_hFontTitle = CreateFontW(26, 0, 0, 0, FW_BOLD, 0, 0, 0, DEFAULT_CHARSET,
                               0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");
    g_hFontBig = CreateFontW(22, 0, 0, 0, FW_BOLD, 0, 0, 0, DEFAULT_CHARSET, 0,
                             0, CLEARTYPE_QUALITY, 0, L"Consolas");
    g_hFontNormal =
        CreateFontW(15, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET, 0, 0,
                    CLEARTYPE_QUALITY, 0, L"Consolas");
    g_hFontSmall = CreateFontW(12, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                               0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");
    g_hFontRain = CreateFontW(14, 0, 0, 0, FW_NORMAL, 0, 0, 0, DEFAULT_CHARSET,
                              0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");

    // Brushes
    g_hBrBg = CreateSolidBrush(CLR_BG);
    g_hBrCard = CreateSolidBrush(CLR_CARD);

    int lx = 20;  // left X
    int cw = 240; // control width
    int y = 75;

    // --- Pitch (Vertical) ---
    CreateWindowW(L"STATIC", L"Pitch (Vertical):", WS_CHILD | WS_VISIBLE, lx, y,
                  140, 18, hwnd, NULL, NULL, NULL);
    g_hPitchVal =
        CreateWindowW(L"STATIC", L"50 px", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                      lx + cw - 50, y, 50, 18, hwnd, NULL, NULL, NULL);
    g_hPitchSlider = CreateWindowW(
        TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | TBS_NOTICKS, lx, y + 18,
        cw, 24, hwnd, (HMENU)IDC_PITCH_SLIDER, NULL, NULL);
    SendMessage(g_hPitchSlider, TBM_SETRANGE, TRUE, MAKELONG(0, 500));
    SendMessage(g_hPitchSlider, TBM_SETPOS, TRUE, 50);

    y += 48;

    // --- Yaw (Horizontal) ---
    CreateWindowW(L"STATIC", L"Yaw (Horizontal):", WS_CHILD | WS_VISIBLE, lx, y,
                  140, 18, hwnd, NULL, NULL, NULL);
    g_hYawVal =
        CreateWindowW(L"STATIC", L"50 px", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                      lx + cw - 50, y, 50, 18, hwnd, NULL, NULL, NULL);
    g_hYawSlider = CreateWindowW(
        TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | TBS_NOTICKS, lx, y + 18,
        cw, 24, hwnd, (HMENU)IDC_YAW_SLIDER, NULL, NULL);
    SendMessage(g_hYawSlider, TBM_SETRANGE, TRUE, MAKELONG(0, 500));
    SendMessage(g_hYawSlider, TBM_SETPOS, TRUE, 50);

    y += 48;

    // --- Interval ---
    CreateWindowW(L"STATIC", L"Interval:", WS_CHILD | WS_VISIBLE, lx, y, 140,
                  18, hwnd, NULL, NULL, NULL);
    g_hIntervalVal =
        CreateWindowW(L"STATIC", L"5 s", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                      lx + cw - 50, y, 50, 18, hwnd, NULL, NULL, NULL);
    g_hIntervalSlider = CreateWindowW(
        TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | TBS_NOTICKS, lx, y + 18,
        cw, 24, hwnd, (HMENU)IDC_INT_SLIDER, NULL, NULL);
    SendMessage(g_hIntervalSlider, TBM_SETRANGE, TRUE, MAKELONG(1, 60));
    SendMessage(g_hIntervalSlider, TBM_SETPOS, TRUE, 5);

    y += 48;

    // --- Start Delay ---
    CreateWindowW(L"STATIC", L"Start Delay:", WS_CHILD | WS_VISIBLE, lx, y, 140,
                  18, hwnd, NULL, NULL, NULL);
    g_hDelayVal =
        CreateWindowW(L"STATIC", L"5 s", WS_CHILD | WS_VISIBLE | SS_RIGHT,
                      lx + cw - 50, y, 50, 18, hwnd, NULL, NULL, NULL);
    g_hDelaySlider = CreateWindowW(
        TRACKBAR_CLASSW, L"", WS_CHILD | WS_VISIBLE | TBS_NOTICKS, lx, y + 18,
        cw, 24, hwnd, (HMENU)IDC_DELAY_SLIDER, NULL, NULL);
    SendMessage(g_hDelaySlider, TBM_SETRANGE, TRUE, MAKELONG(2, 10));
    SendMessage(g_hDelaySlider, TBM_SETPOS, TRUE, 5);

    y += 48;

    // --- Algorithm ---
    CreateWindowW(L"STATIC", L"Algorithm:", WS_CHILD | WS_VISIBLE, lx, y, 80,
                  18, hwnd, NULL, NULL, NULL);
    g_hAlgoCombo = CreateWindowW(
        L"COMBOBOX", L"", WS_CHILD | WS_VISIBLE | CBS_DROPDOWNLIST | WS_VSCROLL,
        lx + 85, y - 2, cw - 85, 150, hwnd, (HMENU)IDC_ALGO_COMBO, NULL, NULL);
    SendMessage(g_hAlgoCombo, CB_ADDSTRING, 0, (LPARAM)L"WindMouse (Classic)");
    SendMessage(g_hAlgoCombo, CB_ADDSTRING, 0, (LPARAM)L"Enhanced WindMouse");
    SendMessage(g_hAlgoCombo, CB_ADDSTRING, 0, (LPARAM)L"Bezier Curves");
    SendMessage(g_hAlgoCombo, CB_ADDSTRING, 0,
                (LPARAM)L"Fitts's Law + Overshoot");
    SendMessage(g_hAlgoCombo, CB_ADDSTRING, 0, (LPARAM)L"AI Engine");
    SendMessage(g_hAlgoCombo, CB_SETCURSEL, 0, 0);

    y += 30;

    // --- Checkboxes ---
    g_hChaosCheck = CreateWindowW(
        L"BUTTON", L"CHAOS MODE!", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX, lx,
        y, 120, 22, hwnd, (HMENU)IDC_CHAOS_CHECK, NULL, NULL);
    g_hTopCheck = CreateWindowW(
        L"BUTTON", L"Always on Top", WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
        lx + 125, y, 115, 22, hwnd, (HMENU)IDC_ONTOP_CHECK, NULL, NULL);

    y += 32;

    // --- START Button ---
    g_hStartBtn = CreateWindowW(L"BUTTON", L"WIGGLE ME!",
                                WS_CHILD | WS_VISIBLE | BS_OWNERDRAW, lx, y, cw,
                                48, hwnd, (HMENU)IDC_START_BTN, NULL, NULL);

    y += 56;

    // --- LEARN Button ---
    g_hRecordBtn = CreateWindowW(
        L"BUTTON", L"LEARN MY MOVEMENT", WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
        lx, y, cw, 32, hwnd, (HMENU)IDC_RECORD_BTN, NULL, NULL);

    y += 38;

    // --- Fingerprint Status ---
    g_hFPStatus = CreateWindowW(L"STATIC", L"[NO FINGERPRINT]",
                                WS_CHILD | WS_VISIBLE | SS_LEFT, lx, y, cw, 18,
                                hwnd, NULL, NULL, NULL);

    y += 24;

    // --- Status ---
    g_hStatus = CreateWindowW(L"STATIC", L">> SYSTEM READY <<",
                              WS_CHILD | WS_VISIBLE | SS_CENTER, lx, y, cw, 18,
                              hwnd, NULL, NULL, NULL);

    // Set font on all children
    g_overrideThreshold = GetDpiScale(hwnd);
    EnumChildWindows(
        hwnd,
        [](HWND hChild, LPARAM lParam) -> BOOL {
          SendMessage(hChild, WM_SETFONT, (WPARAM)lParam, TRUE);
          return TRUE;
        },
        (LPARAM)g_hFontNormal);

    // Auto-load fingerprint from %APPDATA%
    {
      char fpPath[MAX_PATH];
      DWORD len = GetEnvironmentVariableA("APPDATA", fpPath, MAX_PATH);
      if (len > 0 && len < MAX_PATH - 40) {
        strcat_s(fpPath, "\\WiggleMeAI");
        CreateDirectoryA(fpPath, NULL);
        strcat_s(fpPath, "\\fingerprint.dat");
        if (g_engine.loadFingerprint(fpPath)) {
          auto &fp = g_engine.getFingerprint();
          wchar_t buf[128];
          swprintf(buf, 128, L"[LOADED] %.0f px/s", fp.avgSpeed);
          SetWindowTextW(g_hFPStatus, buf);
          SetWindowTextW(g_hStatus, L">> FINGERPRINT LOADED <<");
        }
      }
    }

    return 0;
  }

  case WM_PAINT: {
    PAINTSTRUCT ps;
    HDC hdc = BeginPaint(hwnd, &ps);
    RECT rc;
    GetClientRect(hwnd, &rc);

    // Double buffer
    HDC memDC = CreateCompatibleDC(hdc);
    HBITMAP memBmp = CreateCompatibleBitmap(hdc, rc.right, rc.bottom);
    HBITMAP oldBmp = (HBITMAP)SelectObject(memDC, memBmp);

    // Fill background (pure black)
    FillRect(memDC, &rc, g_hBrBg);

    // === MATRIX DIGITAL RAIN (behind everything) ===
    UpdateAndDrawRain(memDC, rc.right, rc.bottom);

    // Title with Matrix glow
    DrawLabel(memDC, L"WIGGLEME.AI", 20, 12, CLR_RAIN_HEAD, g_hFontTitle);
    DrawLabel(memDC, L"// HUMAN-KINEMATIC ENGINE v2.0", 20, 42, CLR_TEXT_DIM,
              g_hFontSmall);

    // Right panel card (semi-transparent dark green)
    int rx = 280, ry = 75, rw = 200, rh = 420;
    FillRoundRect(memDC, rx, ry, rw, rh, 6, CLR_CARD, CLR_BORDER);

    // Authenticity gauge
    DrawLabel(memDC, L"AUTHENTICITY", rx + 40, ry + 10, CLR_GREEN,
              g_hFontNormal);
    DrawScoreGauge(memDC, rx + rw / 2, ry + 85, 48, g_lastScore.load());

    // Stats
    int sy = ry + 150;
    wchar_t buf[128];

    DrawLabel(memDC, L"[ TELEMETRY ]", rx + 48, sy, CLR_TEXT_DIM, g_hFontSmall);
    sy += 22;

    swprintf(buf, 128, L"> Moves: %d", g_movementCount.load());
    DrawLabel(memDC, buf, rx + 15, sy, CLR_TEXT, g_hFontSmall);
    sy += 18;

    swprintf(buf, 128, L"> FP: %s",
             g_engine.hasFingerprint() ? L"ONLINE" : L"OFFLINE");
    DrawLabel(memDC, buf, rx + 15, sy,
              g_engine.hasFingerprint() ? CLR_GREEN : CLR_RED, g_hFontSmall);
    sy += 18;

    swprintf(buf, 128, L"> Markov: %s",
             g_engine.hasMarkovData() ? L"TRAINED" : L"---");
    DrawLabel(memDC, buf, rx + 15, sy,
              g_engine.hasMarkovData() ? CLR_CYAN : CLR_TEXT_DIM, g_hFontSmall);
    sy += 18;

    swprintf(buf, 128, L"> VelCurves: %d/3", g_engine.getVelocityCurveCount());
    DrawLabel(memDC, buf, rx + 15, sy,
              g_engine.hasVelocityCurves() ? CLR_CYAN : CLR_TEXT_DIM,
              g_hFontSmall);
    sy += 18;

    double score = g_lastScore.load();
    const wchar_t *risk =
        score >= 80 ? L"LOW" : (score >= 60 ? L"MED" : L"HIGH");
    COLORREF riskClr =
        score >= 80 ? CLR_GREEN : (score >= 60 ? CLR_YELLOW : CLR_RED);
    swprintf(buf, 128, L"> Risk: %s", risk);
    DrawLabel(memDC, buf, rx + 15, sy, riskClr, g_hFontSmall);
    sy += 18;

    swprintf(buf, 128, L"> Status: %s",
             g_bRunning.load() ? L"ACTIVE" : L"IDLE");
    DrawLabel(memDC, buf, rx + 15, sy,
              g_bRunning.load() ? CLR_GREEN : CLR_TEXT_DIM, g_hFontSmall);
    sy += 20;

    // Algorithm info
    int algoIdx = g_algorithm.load();
    const wchar_t *algoNames[] = {L"WindMouse", L"EnhancedWM", L"Bezier",
                                  L"Fitts", L"AI-Engine"};
    if (algoIdx >= 0 && algoIdx <= 4) {
      swprintf(buf, 128, L"> Algo: %s", algoNames[algoIdx]);
      DrawLabel(memDC, buf, rx + 15, sy, CLR_CYAN, g_hFontSmall);
    }
    sy += 18;

    swprintf(buf, 128, L"> P:%d Y:%d", g_pitch.load(), g_yaw.load());
    DrawLabel(memDC, buf, rx + 15, sy, CLR_TEXT_DIM, g_hFontSmall);
    sy += 18;

    swprintf(buf, 128, L"> Int:%ds Del:%ds", (int)g_interval.load(),
             g_delay.load());
    DrawLabel(memDC, buf, rx + 15, sy, CLR_TEXT_DIM, g_hFontSmall);

    // === SCANLINE OVERLAY (on top of everything) ===
    DrawScanlines(memDC, rc.right, rc.bottom);

    // Blit
    BitBlt(hdc, 0, 0, rc.right, rc.bottom, memDC, 0, 0, SRCCOPY);
    SelectObject(memDC, oldBmp);
    DeleteObject(memBmp);
    DeleteDC(memDC);

    EndPaint(hwnd, &ps);
    return 0;
  }

  case WM_DRAWITEM: {
    LPDRAWITEMSTRUCT lpDIS = (LPDRAWITEMSTRUCT)lParam;
    if (!lpDIS)
      break;

    HDC hdc = lpDIS->hDC;
    RECT rc = lpDIS->rcItem;
    bool isStart = (lpDIS->CtlID == IDC_START_BTN);
    bool isRecord = (lpDIS->CtlID == IDC_RECORD_BTN);

    COLORREF bgColor, textColor;
    if (isStart) {
      bgColor = g_bRunning.load() ? CLR_RED : CLR_GREEN;
      textColor = RGB(0, 0, 0);
    } else if (isRecord) {
      bgColor = g_bRecording.load() ? CLR_YELLOW : CLR_ACCENT;
      textColor = RGB(255, 255, 255);
      if (g_bRecording.load())
        textColor = RGB(0, 0, 0);
    } else {
      bgColor = CLR_CARD;
      textColor = CLR_TEXT;
    }

    if (lpDIS->itemState & ODS_SELECTED) {
      bgColor =
          RGB(GetRValue(bgColor) * 75 / 100, GetGValue(bgColor) * 75 / 100,
              GetBValue(bgColor) * 75 / 100);
    }

    HBRUSH brush = CreateSolidBrush(bgColor);
    HPEN pen = CreatePen(PS_SOLID, 1, bgColor);
    SelectObject(hdc, brush);
    SelectObject(hdc, pen);
    RoundRect(hdc, rc.left, rc.top, rc.right, rc.bottom, 8, 8);
    DeleteObject(brush);
    DeleteObject(pen);

    wchar_t text[64] = {};
    GetWindowTextW(lpDIS->hwndItem, text, 64);
    SetTextColor(hdc, textColor);
    SetBkMode(hdc, TRANSPARENT);
    SelectObject(hdc, isStart ? g_hFontBig : g_hFontNormal);
    DrawTextW(hdc, text, -1, &rc, DT_CENTER | DT_VCENTER | DT_SINGLELINE);

    return TRUE;
  }

  case WM_CTLCOLORSTATIC:
  case WM_CTLCOLORBTN: {
    HDC hdcCtrl = (HDC)wParam;
    SetTextColor(hdcCtrl, CLR_TEXT);
    SetBkColor(hdcCtrl, CLR_BG);
    SetBkMode(hdcCtrl, TRANSPARENT);
    return (LRESULT)g_hBrBg;
  }

  case WM_HSCROLL: {
    g_pitch = (int)SendMessage(g_hPitchSlider, TBM_GETPOS, 0, 0);
    g_yaw = (int)SendMessage(g_hYawSlider, TBM_GETPOS, 0, 0);
    g_interval = (double)SendMessage(g_hIntervalSlider, TBM_GETPOS, 0, 0);
    g_delay = (int)SendMessage(g_hDelaySlider, TBM_GETPOS, 0, 0);
    UpdateSliderLabels();
    InvalidateRect(hwnd, NULL, FALSE);
    return 0;
  }

  case WM_COMMAND: {
    int cmdId = LOWORD(wParam);

    if (cmdId == IDC_START_BTN) {
      if (!g_bRunning) {
        g_bRunning = true;
        g_algorithm = (int)SendMessage(g_hAlgoCombo, CB_GETCURSEL, 0, 0);
        g_bChaosMode =
            (SendMessage(g_hChaosCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);

        wchar_t lockText[50];
        swprintf(lockText, 50, L"STOP (Lock: %ds)", g_delay.load());
        SetWindowText(g_hStartBtn, lockText);
        EnableWindow(g_hStartBtn, FALSE);
        EnableWindow(g_hAlgoCombo, FALSE);

        // Unlock button after delay
        std::thread(
            [](HWND btn, int delay) {
              Sleep(delay * 1000);
              if (g_bRunning) {
                SetWindowText(btn, L"STOP!");
                EnableWindow(btn, TRUE);
              }
            },
            g_hStartBtn, g_delay.load())
            .detach();

        SetWindowText(g_hStatus, L"Wiggling... Move mouse to stop.");
        std::thread(WigglerLoop, hwnd).detach();
      } else {
        g_bRunning = false;
        SetWindowText(g_hStartBtn, L"WIGGLE ME!");
        SetWindowText(g_hStatus, L"Phew, stopped.");
        EnableWindow(g_hAlgoCombo, TRUE);
      }
      InvalidateRect(hwnd, NULL, FALSE);
    }

    if (cmdId == IDC_RECORD_BTN) {
      if (g_bRecording.load()) {
        g_bRecording.store(false);
        if (g_recordThread.joinable())
          g_recordThread.join();
        SetWindowTextW(g_hRecordBtn, L"LEARN MY MOVEMENT");
        if (g_engine.hasFingerprint()) {
          auto &fp = g_engine.getFingerprint();
          wchar_t buf[128];
          swprintf(buf, 128, L"Fingerprint: Avg %.0f px/s, %d pauses/min",
                   fp.avgSpeed, (int)fp.pauseFrequency);
          SetWindowTextW(g_hFPStatus, buf);
        } else {
          SetWindowTextW(g_hFPStatus, L"Not enough data");
        }
      } else {
        g_bRecording.store(true);
        SetWindowTextW(g_hRecordBtn, L"STOP RECORDING");
        SetWindowTextW(g_hFPStatus, L"Move your mouse naturally...");
        g_recordThread = std::thread(RecordThread);
      }
      InvalidateRect(hwnd, NULL, FALSE);
    }

    if (cmdId == 9999) { // Safety stop from wiggler thread
      g_bRunning = false;
      SetWindowText(g_hStartBtn, L"WIGGLE ME!");
      EnableWindow(g_hStartBtn, TRUE);
      EnableWindow(g_hAlgoCombo, TRUE);
      SetWindowText(g_hStatus, L"Manual override detected!");
      InvalidateRect(hwnd, NULL, FALSE);
    }

    if (cmdId == IDC_CHAOS_CHECK) {
      g_bChaosMode =
          (SendMessage(g_hChaosCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
    }
    if (cmdId == IDC_ONTOP_CHECK) {
      bool onTop = (SendMessage(g_hTopCheck, BM_GETCHECK, 0, 0) == BST_CHECKED);
      SetWindowPos(hwnd, onTop ? HWND_TOPMOST : HWND_NOTOPMOST, 0, 0, 0, 0,
                   SWP_NOMOVE | SWP_NOSIZE);
    }

    return 0;
  }

  case WM_UPDATE_STATUS: {
    wchar_t *statusMsg = (wchar_t *)lParam;
    if (statusMsg) {
      SetWindowText(g_hStatus, statusMsg);
      delete[] statusMsg;
    }
    InvalidateRect(hwnd, NULL, FALSE);
    return 0;
  }

  case WM_SIZE: {
    if (wParam == SIZE_MINIMIZED) {
      AddTrayIcon(hwnd);
      ShowWindow(hwnd, SW_HIDE);
    }
    return 0;
  }

  case WM_TRAYICON: {
    if (lParam == WM_LBUTTONUP || lParam == WM_LBUTTONDBLCLK) {
      ShowWindow(hwnd, SW_RESTORE);
      SetForegroundWindow(hwnd);
      RemoveTrayIcon();
    }
    return 0;
  }

  case WM_CLOSE:
    g_bRunning = false;
    g_bRecording.store(false);
    if (g_recordThread.joinable())
      g_recordThread.join();
    // Auto-save fingerprint on close
    if (g_engine.hasFingerprint()) {
      char fpPath[MAX_PATH];
      DWORD len = GetEnvironmentVariableA("APPDATA", fpPath, MAX_PATH);
      if (len > 0 && len < MAX_PATH - 40) {
        strcat_s(fpPath, "\\WiggleMeAI");
        CreateDirectoryA(fpPath, NULL);
        strcat_s(fpPath, "\\fingerprint.dat");
        g_engine.saveFingerprint(fpPath);
      }
    }
    RemoveTrayIcon();
    DestroyWindow(hwnd);
    return 0;

  case WM_DESTROY:
    if (g_hFontTitle)
      DeleteObject(g_hFontTitle);
    if (g_hFontBig)
      DeleteObject(g_hFontBig);
    if (g_hFontNormal)
      DeleteObject(g_hFontNormal);
    if (g_hFontSmall)
      DeleteObject(g_hFontSmall);
    if (g_hFontRain)
      DeleteObject(g_hFontRain);
    if (g_hBrBg)
      DeleteObject(g_hBrBg);
    if (g_hBrCard)
      DeleteObject(g_hBrCard);
    PostQuitMessage(0);
    return 0;
  }

  return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ============================================================================
// Entry Point
// ============================================================================
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int nShow) {
  INITCOMMONCONTROLSEX icex = {sizeof(icex),
                               ICC_BAR_CLASSES | ICC_STANDARD_CLASSES};
  InitCommonControlsEx(&icex);
  srand((unsigned)time(0));

  WNDCLASSEXW wc = {};
  wc.cbSize = sizeof(wc);
  wc.lpfnWndProc = WndProc;
  wc.hInstance = hInst;
  wc.hCursor = LoadCursor(NULL, IDC_ARROW);
  wc.hbrBackground = CreateSolidBrush(CLR_BG);
  wc.lpszClassName = L"WiggleMeAI";
  wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
  RegisterClassExW(&wc);

  int winW = 510;
  int winH = 550;
  int screenW = GetSystemMetrics(SM_CXSCREEN);
  int screenH = GetSystemMetrics(SM_CYSCREEN);

  HWND hwnd =
      CreateWindowExW(WS_EX_APPWINDOW, L"WiggleMeAI", L"WiggleMe AI",
                      WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
                      (screenW - winW) / 2, (screenH - winH) / 2, winW, winH,
                      NULL, NULL, hInst, NULL);

  ShowWindow(hwnd, nShow);
  UpdateWindow(hwnd);

  // Animation timer (30fps for Matrix rain)
  SetTimer(hwnd, 1, 33, NULL);

  MSG msgLoop;
  while (GetMessage(&msgLoop, NULL, 0, 0)) {
    if (msgLoop.message == WM_TIMER && msgLoop.wParam == 1) {
      InvalidateRect(hwnd, NULL, FALSE);
    }
    TranslateMessage(&msgLoop);
    DispatchMessage(&msgLoop);
  }

  return (int)msgLoop.wParam;
}
