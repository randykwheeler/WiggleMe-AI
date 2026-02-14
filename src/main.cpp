// ============================================================================
// WiggleMe AI - Human-Kinematic Mouse Simulator with AI Movement Engine
// A workplace activity tool with behavioral fingerprinting
// ============================================================================

#include <windows.h>
#include <commctrl.h>
#include <shellapi.h>
#include <tlhelp32.h>
#include <string>
#include <thread>
#include <atomic>
#include <mutex>
#include <sstream>
#include <iomanip>

#include "../include/ai_movement.h"

#pragma comment(lib, "comctl32.lib")
#pragma comment(lib, "user32.lib")
#pragma comment(lib, "gdi32.lib")
#pragma comment(lib, "shell32.lib")
#pragma comment(lib, "shlwapi.lib")

// ============================================================================
// Constants & IDs
// ============================================================================
#define APP_NAME        L"WiggleMe AI"
#define APP_VERSION     L"1.0.0-proto"
#define WM_TRAYICON     (WM_APP + 1)
#define WM_UPDATE_UI    (WM_APP + 2)
#define IDI_TRAYICON    1001
#define IDC_START_BTN   2001
#define IDC_RECORD_BTN  2002
#define IDC_INTENSITY   2003
#define IDC_TEMPORAL    2004
#define IDC_MEETING     2005
#define IDC_STEALTH     2006
#define IDC_STATUS      2007
#define IDC_SCORE       2008
#define IDC_SCHEDULE    2009

// ============================================================================
// Colors (Modern dark theme)
// ============================================================================
#define CLR_BG          RGB(30, 30, 46)     // Dark background
#define CLR_SURFACE     RGB(45, 45, 65)     // Card/panel background
#define CLR_ACCENT      RGB(0, 188, 212)    // Cyan accent
#define CLR_ACCENT2     RGB(139, 92, 246)   // Purple accent
#define CLR_TEXT        RGB(205, 214, 244)   // Light text
#define CLR_TEXT_DIM    RGB(147, 153, 178)   // Dimmed text
#define CLR_GREEN       RGB(76, 175, 80)    // Active/good
#define CLR_YELLOW      RGB(255, 193, 7)    // Warning
#define CLR_RED         RGB(244, 67, 54)    // Danger/stopped
#define CLR_BORDER      RGB(60, 60, 85)     // Border color

// ============================================================================
// Globals
// ============================================================================
static HWND g_hWnd = NULL;
static HWND g_hStartBtn, g_hRecordBtn;
static HWND g_hIntensitySlider, g_hIntensityLabel;
static HWND g_hTemporalCheck, g_hMeetingCheck, g_hStealthCheck;
static HWND g_hStatus, g_hScoreLabel, g_hScheduleLabel;
static HWND g_hFingerprintStatus;

static HFONT g_hFontTitle = NULL;
static HFONT g_hFontNormal = NULL;
static HFONT g_hFontSmall = NULL;
static HFONT g_hFontMono = NULL;
static HBRUSH g_hBrBg = NULL;
static HBRUSH g_hBrSurface = NULL;

static NOTIFYICONDATA g_nid = {};
static bool g_bTrayActive = false;

static AIMovementEngine g_engine;
static std::atomic<bool> g_bRunning(false);
static std::atomic<bool> g_bRecording(false);
static std::thread g_workerThread;
static std::thread g_recordThread;
static std::mutex g_mutex;

static std::atomic<double> g_lastScore(0.0);
static std::atomic<int> g_movementCount(0);
static std::wstring g_statusText = L"Idle";

// ============================================================================
// Meeting Detection
// ============================================================================
static bool IsInMeeting() {
    // Check if common video conferencing apps have active windows
    const wchar_t* meetingApps[] = {
        L"Teams.exe", L"ms-teams.exe", L"Zoom.exe", L"zoom.exe",
        L"webex.exe", L"CiscoCollabHost.exe",
        L"slack.exe", L"Discord.exe",
        L"GoogleMeet",
        nullptr
    };

    HANDLE snap = CreateToolhelp32Snapshot(TH32CS_SNAPPROCESS, 0);
    if (snap == INVALID_HANDLE_VALUE) return false;

    PROCESSENTRY32W pe;
    pe.dwSize = sizeof(pe);
    bool found = false;

    if (Process32FirstW(snap, &pe)) {
        do {
            for (int i = 0; meetingApps[i]; i++) {
                if (wcsstr(pe.szExeFile, meetingApps[i])) {
                    // App is running - check if it has a foreground window
                    // (rough heuristic: if meeting app is running, assume meeting)
                    HWND fg = GetForegroundWindow();
                    DWORD fgPid = 0;
                    GetWindowThreadProcessId(fg, &fgPid);
                    if (fgPid == pe.th32ProcessID) {
                        found = true;
                        break;
                    }
                }
            }
            if (found) break;
        } while (Process32NextW(snap, &pe));
    }

    CloseHandle(snap);
    return found;
}

// ============================================================================
// Manual Override Detection
// ============================================================================
static POINT g_lastKnownPos = {0, 0};
static bool DetectManualOverride() {
    POINT curPos;
    GetCursorPos(&curPos);

    // If cursor moved more than threshold since we last set it,
    // user is actively using the mouse
    double dist = sqrt(
        (double)(curPos.x - g_lastKnownPos.x) * (curPos.x - g_lastKnownPos.x) +
        (double)(curPos.y - g_lastKnownPos.y) * (curPos.y - g_lastKnownPos.y)
    );

    return (dist > 30.0); // 30px threshold
}

// ============================================================================
// Worker Thread - AI Movement Execution
// ============================================================================
static void WorkerThread() {
    POINT startPos;
    GetCursorPos(&startPos);
    Point2D currentPos((double)startPos.x, (double)startPos.y);

    while (g_bRunning.load()) {
        // Check temporal awareness
        bool temporalEnabled = (IsDlgButtonChecked(g_hWnd, IDC_TEMPORAL) == BST_CHECKED);
        if (temporalEnabled && !g_engine.shouldBeActive()) {
            g_statusText = L"Outside work hours - paused";
            PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
            Sleep(30000); // Check every 30s
            continue;
        }

        // Check meeting detection
        bool meetingEnabled = (IsDlgButtonChecked(g_hWnd, IDC_MEETING) == BST_CHECKED);
        if (meetingEnabled && IsInMeeting()) {
            g_statusText = L"Meeting detected - paused";
            PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
            Sleep(10000); // Check every 10s
            continue;
        }

        // Check manual override
        if (DetectManualOverride()) {
            g_statusText = L"User active - standing by";
            PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
            // Wait for user to stop moving
            Sleep(5000);
            GetCursorPos(&startPos);
            currentPos = Point2D((double)startPos.x, (double)startPos.y);
            continue;
        }

        // Generate activity session (30-90 seconds)
        double sessionDuration = 30.0 + (rand() % 60);
        auto session = g_engine.generateActivitySession(currentPos, sessionDuration);

        for (auto& tm : session) {
            if (!g_bRunning.load()) break;

            // Check for manual override during execution
            if (DetectManualOverride()) break;

            // Wait before movement (natural pause)
            int waitMs = (int)tm.delayBeforeMs;
            int waited = 0;
            while (waited < waitMs && g_bRunning.load()) {
                Sleep(100);
                waited += 100;
                if (DetectManualOverride()) break;
            }
            if (!g_bRunning.load() || DetectManualOverride()) break;

            // Generate and execute trajectory
            auto trajectory = g_engine.generateTrajectory(currentPos, tm.target);

            // Calculate authenticity score for this movement
            double score = g_engine.calculateAuthenticityScore(trajectory);
            g_lastScore.store(score);

            // Execute movement with realistic timing
            for (size_t i = 0; i < trajectory.size(); i++) {
                if (!g_bRunning.load()) break;

                SetCursorPos((int)trajectory[i].x, (int)trajectory[i].y);
                g_lastKnownPos = {(LONG)trajectory[i].x, (LONG)trajectory[i].y};

                // Variable delay between points (speed variation)
                double baseDelay = 1.0; // 1ms base
                if (i > 0) {
                    double dist = trajectory[i].distTo(trajectory[i-1]);
                    double speed = g_engine.getFingerprint().isValid ?
                        g_engine.getFingerprint().avgSpeed : 350.0;
                    baseDelay = (dist / speed) * 1000.0; // Convert to ms
                }
                if (baseDelay > 0.5) Sleep((DWORD)(std::max)(1.0, baseDelay));
            }

            // Optional click
            if (tm.includeClick && !trajectory.empty()) {
                mouse_event(MOUSEEVENTF_LEFTDOWN, 0, 0, 0, 0);
                Sleep(50 + rand() % 80); // Natural click duration
                mouse_event(MOUSEEVENTF_LEFTUP, 0, 0, 0, 0);
            }

            if (!trajectory.empty()) {
                currentPos = trajectory.back();
            }

            g_movementCount++;
            g_statusText = L"Active - generating movement";
            PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
        }

        // Brief pause between sessions
        Sleep(2000 + rand() % 3000);
    }

    g_statusText = L"Stopped";
    PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
}

// ============================================================================
// Recording Thread - Fingerprint Learning
// ============================================================================
static void RecordThread() {
    g_engine.startRecording();
    auto startTime = std::chrono::steady_clock::now();

    while (g_bRecording.load()) {
        POINT pos;
        GetCursorPos(&pos);
        bool clicking = (GetAsyncKeyState(VK_LBUTTON) & 0x8000) != 0;
        g_engine.addSample(Point2D((double)pos.x, (double)pos.y), clicking);

        auto elapsed = std::chrono::steady_clock::now() - startTime;
        double secs = std::chrono::duration<double>(elapsed).count();

        std::wstringstream ss;
        ss << L"Recording: " << std::fixed << std::setprecision(0) << secs << L"s";
        g_statusText = ss.str();
        PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);

        Sleep(16); // ~60 samples/second
    }

    g_engine.stopRecording();
    g_engine.buildFingerprint();

    if (g_engine.hasFingerprint()) {
        g_statusText = L"Fingerprint learned!";
    } else {
        g_statusText = L"Not enough data - record longer";
    }
    PostMessage(g_hWnd, WM_UPDATE_UI, 0, 0);
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
    wcscpy_s(g_nid.szTip, APP_NAME);
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
// Custom Drawing
// ============================================================================
static void DrawRoundRect(HDC hdc, int x, int y, int w, int h, int r, COLORREF fill) {
    HBRUSH brush = CreateSolidBrush(fill);
    HPEN pen = CreatePen(PS_SOLID, 1, CLR_BORDER);
    SelectObject(hdc, brush);
    SelectObject(hdc, pen);
    RoundRect(hdc, x, y, x + w, y + h, r, r);
    DeleteObject(brush);
    DeleteObject(pen);
}

static void DrawText_(HDC hdc, const wchar_t* text, int x, int y,
    COLORREF color, HFONT font) {
    SetTextColor(hdc, color);
    SetBkMode(hdc, TRANSPARENT);
    SelectObject(hdc, font);
    TextOutW(hdc, x, y, text, (int)wcslen(text));
}

static void DrawScoreGauge(HDC hdc, int cx, int cy, int radius, double score) {
    // Draw background circle
    HBRUSH bgBrush = CreateSolidBrush(CLR_SURFACE);
    HPEN bgPen = CreatePen(PS_SOLID, 3, CLR_BORDER);
    SelectObject(hdc, bgBrush);
    SelectObject(hdc, bgPen);
    Ellipse(hdc, cx - radius, cy - radius, cx + radius, cy + radius);
    DeleteObject(bgBrush);
    DeleteObject(bgPen);

    // Draw score arc
    COLORREF arcColor = CLR_RED;
    if (score >= 80) arcColor = CLR_GREEN;
    else if (score >= 60) arcColor = CLR_YELLOW;

    HPEN arcPen = CreatePen(PS_SOLID, 6, arcColor);
    SelectObject(hdc, arcPen);
    SelectObject(hdc, GetStockObject(NULL_BRUSH));

    double startAngle = 3.14159 * 0.75;  // Start at 7 o'clock
    double endAngle = startAngle + (score / 100.0) * 3.14159 * 1.5;

    // Draw arc using segments
    int segments = (int)(score / 2);
    for (int i = 0; i < segments; i++) {
        double a1 = startAngle + (double)i / segments * (endAngle - startAngle);
        double a2 = startAngle + (double)(i + 1) / segments * (endAngle - startAngle);
        int x1 = cx + (int)(cos(a1) * (radius - 5));
        int y1 = cy - (int)(sin(a1) * (radius - 5));
        int x2 = cx + (int)(cos(a2) * (radius - 5));
        int y2 = cy - (int)(sin(a2) * (radius - 5));
        MoveToEx(hdc, x1, y1, NULL);
        LineTo(hdc, x2, y2);
    }
    DeleteObject(arcPen);

    // Draw score text
    wchar_t scoreText[32];
    swprintf_s(scoreText, L"%d", (int)score);
    SetTextColor(hdc, CLR_TEXT);
    SetBkMode(hdc, TRANSPARENT);
    SelectObject(hdc, g_hFontTitle);
    SIZE sz;
    GetTextExtentPoint32W(hdc, scoreText, (int)wcslen(scoreText), &sz);
    TextOutW(hdc, cx - sz.cx / 2, cy - sz.cy / 2 - 5, scoreText, (int)wcslen(scoreText));

    // Draw "/ 100" below
    SelectObject(hdc, g_hFontSmall);
    SetTextColor(hdc, CLR_TEXT_DIM);
    TextOutW(hdc, cx - 12, cy + 12, L"/ 100", 5);
}

// ============================================================================
// Window Procedure
// ============================================================================
static LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE: {
        g_hWnd = hwnd;

        // Create fonts
        g_hFontTitle = CreateFontW(28, 0, 0, 0, FW_BOLD, 0, 0, 0,
            DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
        g_hFontNormal = CreateFontW(16, 0, 0, 0, FW_NORMAL, 0, 0, 0,
            DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
        g_hFontSmall = CreateFontW(13, 0, 0, 0, FW_NORMAL, 0, 0, 0,
            DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Segoe UI");
        g_hFontMono = CreateFontW(14, 0, 0, 0, FW_NORMAL, 0, 0, 0,
            DEFAULT_CHARSET, 0, 0, CLEARTYPE_QUALITY, 0, L"Consolas");

        // Create brushes
        g_hBrBg = CreateSolidBrush(CLR_BG);
        g_hBrSurface = CreateSolidBrush(CLR_SURFACE);

        // --- Controls ---
        int y = 80;
        int leftCol = 20;
        int rightCol = 280;

        // Start/Stop Button
        g_hStartBtn = CreateWindowW(L"BUTTON", L"START",
            WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
            leftCol, y, 230, 45, hwnd, (HMENU)IDC_START_BTN, NULL, NULL);

        // Record Fingerprint Button
        g_hRecordBtn = CreateWindowW(L"BUTTON", L"LEARN MY MOVEMENT",
            WS_CHILD | WS_VISIBLE | BS_OWNERDRAW,
            leftCol, y + 55, 230, 35, hwnd, (HMENU)IDC_RECORD_BTN, NULL, NULL);

        // Fingerprint status
        g_hFingerprintStatus = CreateWindowW(L"STATIC", L"No fingerprint recorded",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            leftCol, y + 95, 230, 20, hwnd, NULL, NULL, NULL);

        y += 130;

        // Intensity slider
        CreateWindowW(L"STATIC", L"Movement Intensity",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            leftCol, y, 200, 20, hwnd, NULL, NULL, NULL);
        g_hIntensitySlider = CreateWindowW(TRACKBAR_CLASSW, NULL,
            WS_CHILD | WS_VISIBLE | TBS_AUTOTICKS,
            leftCol, y + 22, 200, 30, hwnd, (HMENU)IDC_INTENSITY, NULL, NULL);
        SendMessage(g_hIntensitySlider, TBM_SETRANGE, TRUE, MAKELPARAM(1, 10));
        SendMessage(g_hIntensitySlider, TBM_SETPOS, TRUE, 7);
        g_hIntensityLabel = CreateWindowW(L"STATIC", L"70%",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            leftCol + 210, y + 25, 40, 20, hwnd, NULL, NULL, NULL);

        y += 60;

        // Checkboxes
        g_hTemporalCheck = CreateWindowW(L"BUTTON", L"Work Hours Only (9-5)",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
            leftCol, y, 230, 24, hwnd, (HMENU)IDC_TEMPORAL, NULL, NULL);
        CheckDlgButton(hwnd, IDC_TEMPORAL, BST_CHECKED);

        g_hMeetingCheck = CreateWindowW(L"BUTTON", L"Pause During Meetings",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
            leftCol, y + 28, 230, 24, hwnd, (HMENU)IDC_MEETING, NULL, NULL);
        CheckDlgButton(hwnd, IDC_MEETING, BST_CHECKED);

        g_hStealthCheck = CreateWindowW(L"BUTTON", L"Detection Avoidance",
            WS_CHILD | WS_VISIBLE | BS_AUTOCHECKBOX,
            leftCol, y + 56, 230, 24, hwnd, (HMENU)IDC_STEALTH, NULL, NULL);
        CheckDlgButton(hwnd, IDC_STEALTH, BST_CHECKED);

        y += 95;

        // Status label
        g_hStatus = CreateWindowW(L"STATIC", L"Status: Idle",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            leftCol, y, 230, 20, hwnd, (HMENU)IDC_STATUS, NULL, NULL);

        g_hScheduleLabel = CreateWindowW(L"STATIC", L"",
            WS_CHILD | WS_VISIBLE | SS_LEFT,
            leftCol, y + 22, 230, 20, hwnd, (HMENU)IDC_SCHEDULE, NULL, NULL);

        // Set fonts on all controls
        HWND controls[] = {
            g_hStartBtn, g_hRecordBtn, g_hFingerprintStatus,
            g_hIntensitySlider, g_hIntensityLabel,
            g_hTemporalCheck, g_hMeetingCheck, g_hStealthCheck,
            g_hStatus, g_hScheduleLabel
        };
        for (auto ctrl : controls) {
            SendMessage(ctrl, WM_SETFONT, (WPARAM)g_hFontNormal, TRUE);
        }

        // Add tray icon
        AddTrayIcon(hwnd);

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
        SelectObject(memDC, memBmp);

        // Fill background
        FillRect(memDC, &rc, g_hBrBg);

        // Title bar area
        DrawText_(memDC, APP_NAME, 20, 15, CLR_ACCENT, g_hFontTitle);
        DrawText_(memDC, APP_VERSION, 220, 25, CLR_TEXT_DIM, g_hFontSmall);

        // Subtitle
        DrawText_(memDC, L"AI-Powered Human Movement Simulation",
            20, 48, CLR_TEXT_DIM, g_hFontSmall);

        // Right panel - Authenticity Score gauge
        int gaugeX = 370;
        int gaugeY = 120;
        DrawText_(memDC, L"Authenticity Score", gaugeX - 20, 80, CLR_TEXT, g_hFontNormal);
        DrawScoreGauge(memDC, gaugeX + 50, gaugeY + 60, 55, g_lastScore.load());

        // Right panel - Stats
        int statsY = 240;
        DrawRoundRect(memDC, gaugeX - 30, statsY, 190, 120, 8, CLR_SURFACE);

        wchar_t buf[128];
        swprintf_s(buf, L"Movements: %d", g_movementCount.load());
        DrawText_(memDC, buf, gaugeX - 15, statsY + 10, CLR_TEXT, g_hFontSmall);

        swprintf_s(buf, L"Fingerprint: %s",
            g_engine.hasFingerprint() ? L"Active" : L"None");
        DrawText_(memDC, buf, gaugeX - 15, statsY + 30, CLR_TEXT, g_hFontSmall);

        swprintf_s(buf, L"Detection Risk: %s",
            g_lastScore.load() >= 80 ? L"LOW" :
            g_lastScore.load() >= 60 ? L"MEDIUM" : L"HIGH");
        COLORREF riskColor = g_lastScore.load() >= 80 ? CLR_GREEN :
            g_lastScore.load() >= 60 ? CLR_YELLOW : CLR_RED;
        DrawText_(memDC, buf, gaugeX - 15, statsY + 50, riskColor, g_hFontSmall);

        swprintf_s(buf, L"Engine: %s",
            g_bRunning.load() ? L"Running" : L"Stopped");
        DrawText_(memDC, buf, gaugeX - 15, statsY + 70,
            g_bRunning.load() ? CLR_GREEN : CLR_RED, g_hFontSmall);

        bool temporal = (IsDlgButtonChecked(hwnd, IDC_TEMPORAL) == BST_CHECKED);
        if (temporal) {
            swprintf_s(buf, L"Intensity: %.0f%%", g_engine.getCurrentIntensity() * 100);
            DrawText_(memDC, buf, gaugeX - 15, statsY + 90, CLR_TEXT_DIM, g_hFontSmall);
        }

        // Blit
        BitBlt(hdc, 0, 0, rc.right, rc.bottom, memDC, 0, 0, SRCCOPY);
        DeleteObject(memBmp);
        DeleteDC(memDC);

        EndPaint(hwnd, &ps);
        return 0;
    }

    case WM_DRAWITEM: {
        LPDRAWITEMSTRUCT lpDIS = (LPDRAWITEMSTRUCT)lParam;
        if (!lpDIS) break;

        HDC hdc = lpDIS->hDC;
        RECT rc = lpDIS->rcItem;

        bool isStart = (lpDIS->CtlID == IDC_START_BTN);
        bool isRecord = (lpDIS->CtlID == IDC_RECORD_BTN);

        COLORREF bgColor, textColor;

        if (isStart) {
            if (g_bRunning.load()) {
                bgColor = CLR_RED; textColor = RGB(255, 255, 255);
            } else {
                bgColor = CLR_GREEN; textColor = RGB(0, 0, 0);
            }
        } else if (isRecord) {
            if (g_bRecording.load()) {
                bgColor = CLR_YELLOW; textColor = RGB(0, 0, 0);
            } else {
                bgColor = CLR_ACCENT2; textColor = RGB(255, 255, 255);
            }
        } else {
            bgColor = CLR_SURFACE; textColor = CLR_TEXT;
        }

        // Hover effect
        if (lpDIS->itemState & ODS_SELECTED) {
            bgColor = RGB(
                GetRValue(bgColor) * 80 / 100,
                GetGValue(bgColor) * 80 / 100,
                GetBValue(bgColor) * 80 / 100
            );
        }

        HBRUSH brush = CreateSolidBrush(bgColor);
        HPEN pen = CreatePen(PS_SOLID, 1, bgColor);
        SelectObject(hdc, brush);
        SelectObject(hdc, pen);
        RoundRect(hdc, rc.left, rc.top, rc.right, rc.bottom, 8, 8);
        DeleteObject(brush);
        DeleteObject(pen);

        // Button text
        wchar_t text[64] = {};
        GetWindowTextW(lpDIS->hwndItem, text, 64);
        SetTextColor(hdc, textColor);
        SetBkMode(hdc, TRANSPARENT);
        SelectObject(hdc, isStart ? g_hFontTitle : g_hFontNormal);
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
        if ((HWND)lParam == g_hIntensitySlider) {
            int pos = (int)SendMessage(g_hIntensitySlider, TBM_GETPOS, 0, 0);
            double intensity = pos / 10.0;
            g_engine.setIntensity(intensity);
            wchar_t buf[16];
            swprintf_s(buf, L"%d%%", pos * 10);
            SetWindowTextW(g_hIntensityLabel, buf);
        }
        return 0;
    }

    case WM_COMMAND: {
        int id = LOWORD(wParam);

        if (id == IDC_START_BTN) {
            if (g_bRunning.load()) {
                // Stop
                g_bRunning.store(false);
                if (g_workerThread.joinable()) g_workerThread.join();
                SetWindowTextW(g_hStartBtn, L"START");
                g_statusText = L"Stopped";
            } else {
                // Start
                g_bRunning.store(true);
                SetWindowTextW(g_hStartBtn, L"STOP");
                g_statusText = L"Starting...";
                g_workerThread = std::thread(WorkerThread);
            }
            InvalidateRect(hwnd, NULL, FALSE);
        }

        if (id == IDC_RECORD_BTN) {
            if (g_bRecording.load()) {
                // Stop recording
                g_bRecording.store(false);
                if (g_recordThread.joinable()) g_recordThread.join();
                SetWindowTextW(g_hRecordBtn, L"LEARN MY MOVEMENT");

                if (g_engine.hasFingerprint()) {
                    auto& fp = g_engine.getFingerprint();
                    wchar_t buf[128];
                    swprintf_s(buf, L"Fingerprint: Avg %.0f px/s, %d pauses/min",
                        fp.avgSpeed, (int)fp.pauseFrequency);
                    SetWindowTextW(g_hFingerprintStatus, buf);
                } else {
                    SetWindowTextW(g_hFingerprintStatus, L"Not enough data");
                }
            } else {
                // Start recording
                g_bRecording.store(true);
                SetWindowTextW(g_hRecordBtn, L"STOP RECORDING");
                SetWindowTextW(g_hFingerprintStatus, L"Move your mouse naturally...");
                g_recordThread = std::thread(RecordThread);
            }
            InvalidateRect(hwnd, NULL, FALSE);
        }

        return 0;
    }

    case WM_UPDATE_UI: {
        SetWindowTextW(g_hStatus, (L"Status: " + g_statusText).c_str());
        InvalidateRect(hwnd, NULL, FALSE);
        return 0;
    }

    case WM_TRAYICON: {
        if (lParam == WM_LBUTTONDBLCLK) {
            ShowWindow(hwnd, SW_RESTORE);
            SetForegroundWindow(hwnd);
        }
        return 0;
    }

    case WM_SIZE: {
        if (wParam == SIZE_MINIMIZED) {
            ShowWindow(hwnd, SW_HIDE);
        }
        return 0;
    }

    case WM_CLOSE:
        g_bRunning.store(false);
        g_bRecording.store(false);
        if (g_workerThread.joinable()) g_workerThread.join();
        if (g_recordThread.joinable()) g_recordThread.join();
        RemoveTrayIcon();
        DestroyWindow(hwnd);
        return 0;

    case WM_DESTROY:
        if (g_hFontTitle) DeleteObject(g_hFontTitle);
        if (g_hFontNormal) DeleteObject(g_hFontNormal);
        if (g_hFontSmall) DeleteObject(g_hFontSmall);
        if (g_hFontMono) DeleteObject(g_hFontMono);
        if (g_hBrBg) DeleteObject(g_hBrBg);
        if (g_hBrSurface) DeleteObject(g_hBrSurface);
        PostQuitMessage(0);
        return 0;
    }

    return DefWindowProcW(hwnd, msg, wParam, lParam);
}

// ============================================================================
// Entry Point
// ============================================================================
int WINAPI wWinMain(HINSTANCE hInst, HINSTANCE, LPWSTR, int nShow) {
    // Enable visual styles
    INITCOMMONCONTROLSEX icex = { sizeof(icex), ICC_BAR_CLASSES | ICC_STANDARD_CLASSES };
    InitCommonControlsEx(&icex);

    // Register window class
    WNDCLASSEXW wc = {};
    wc.cbSize = sizeof(wc);
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInst;
    wc.hCursor = LoadCursor(NULL, IDC_ARROW);
    wc.hbrBackground = CreateSolidBrush(CLR_BG);
    wc.lpszClassName = L"WiggleMeAI";
    wc.hIcon = LoadIcon(NULL, IDI_APPLICATION);
    RegisterClassExW(&wc);

    // Create window (fixed size, modern dimensions)
    int winW = 560;
    int winH = 500;
    int screenW = GetSystemMetrics(SM_CXSCREEN);
    int screenH = GetSystemMetrics(SM_CYSCREEN);

    HWND hwnd = CreateWindowExW(
        WS_EX_APPWINDOW,
        L"WiggleMeAI", APP_NAME,
        WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
        (screenW - winW) / 2, (screenH - winH) / 2,
        winW, winH,
        NULL, NULL, hInst, NULL
    );

    ShowWindow(hwnd, nShow);
    UpdateWindow(hwnd);

    // Set refresh timer (update UI every 2 seconds)
    SetTimer(hwnd, 1, 2000, NULL);

    // Message loop
    MSG msg;
    while (GetMessage(&msg, NULL, 0, 0)) {
        if (msg.message == WM_TIMER && msg.wParam == 1) {
            InvalidateRect(hwnd, NULL, FALSE);
        }
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }

    return (int)msg.wParam;
}
