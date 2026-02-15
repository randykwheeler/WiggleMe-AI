#![windows_subsystem = "windows"]
#![allow(static_mut_refs)]

use noise::{NoiseFn, Perlin};
use rand::Rng;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};
use windows::core::*;
use windows::Win32::Foundation::*;
use windows::Win32::Graphics::Gdi::*;
use windows::Win32::System::LibraryLoader::GetModuleHandleW;
use windows::Win32::UI::Controls::*;
use windows::Win32::UI::Input::KeyboardAndMouse::*;
use windows::Win32::UI::WindowsAndMessaging::*;

const WM_APP_UPDATE: u32 = WM_APP + 1;
const IDC_BTN_START: i32 = 101;
const IDC_BTN_LEARN: i32 = 102;
const IDC_STATIC_SCORE: i32 = 201;
const IDC_STATIC_RISK: i32 = 202;
const IDC_STATIC_MOVES: i32 = 203;
const IDC_STATIC_STATUS: i32 = 204;
const MY_SS_CENTER: u32 = 0x0001;

const BG_COLOR: u32 = 0x00503e2c;
const TEXT_COLOR: u32 = 0x00f0f0f0;
const GREEN_COLOR: u32 = 0x0050c878;
const YELLOW_COLOR: u32 = 0x0000d4f5;
const RED_COLOR: u32 = 0x004040e0;

#[derive(Clone, Copy, Default)]
struct Point2D { x: f64, y: f64 }

#[derive(Clone, Default)]
struct MovementSample {
    pos: Point2D,
    timestamp: f64,
}

#[derive(Clone)]
struct UserFingerprint {
    avg_speed: f64,
    speed_variance: f64,
    preferred_directions: [f64; 8],
    built: bool,
}

impl Default for UserFingerprint {
    fn default() -> Self {
        Self {
            avg_speed: 450.0,
            speed_variance: 150.0,
            preferred_directions: [1.0; 8],
            built: false,
        }
    }
}

struct TemporalProfile {
    hourly_intensity: [f64; 24],
}

impl Default for TemporalProfile {
    fn default() -> Self {
        let mut h = [0.0f64; 24];
        h[8] = 0.6; h[9] = 0.9; h[10] = 1.0; h[11] = 0.95;
        h[12] = 0.3; h[13] = 0.5; h[14] = 0.85; h[15] = 0.9;
        h[16] = 0.8; h[17] = 0.4;
        Self { hourly_intensity: h }
    }
}

impl TemporalProfile {
    fn get_intensity(&self) -> f64 {
        let hour = chrono::Local::now().hour() as usize;
        if hour < 24 { self.hourly_intensity[hour].max(0.1) } else { 0.1 }
    }
}

use chrono::Timelike;

#[derive(Clone, Default)]
struct AuthenticityScore {
    overall: f64,
    speed_variance_score: f64,
    curve_smoothness: f64,
    micro_jitter_score: f64,
    acceleration_pattern: f64,
}

impl AuthenticityScore {
    fn risk_level(&self) -> &'static str {
        if self.overall >= 80.0 { "LOW" }
        else if self.overall >= 50.0 { "MEDIUM" }
        else { "HIGH" }
    }
}

struct AIMovementEngine {
    perlin: Perlin,
    fingerprint: UserFingerprint,
    temporal: TemporalProfile,
    noise_offset: f64,
}

impl AIMovementEngine {
    fn new() -> Self {
        Self {
            perlin: Perlin::new(42),
            fingerprint: UserFingerprint::default(),
            temporal: TemporalProfile::default(),
            noise_offset: 0.0,
        }
    }

    fn set_fingerprint(&mut self, fp: UserFingerprint) {
        self.fingerprint = fp;
    }

    fn wind_mouse(&self, start: Point2D, end: Point2D) -> Vec<Point2D> {
        let mut rng = rand::thread_rng();
        let mut points = Vec::new();
        let dist = ((end.x - start.x).powi(2) + (end.y - start.y).powi(2)).sqrt();
        if dist < 1.0 { return vec![end]; }

        let wind_force = dist.min(150.0) * 0.4;
        let gravity = 9.0 + rng.gen::<f64>() * 3.0;
        let max_step = dist.max(15.0).min(30.0);
        let (mut cx, mut cy) = (start.x, start.y);
        let (mut vx, mut vy) = (0.0f64, 0.0f64);
        let (mut wx, mut wy) = (0.0f64, 0.0f64);

        for step in 0..2000 {
            let remaining = ((end.x - cx).powi(2) + (end.y - cy).powi(2)).sqrt();
            if remaining < 3.0 { points.push(end); break; }

            let nv = self.perlin.get([self.noise_offset + step as f64 * 0.05, 0.0]);
            wx = wx / 2.0 + (rng.gen::<f64>() - 0.5) * wind_force + nv * wind_force * 0.3;
            wy = wy / 2.0 + (rng.gen::<f64>() - 0.5) * wind_force;

            let angle = (end.y - cy).atan2(end.x - cx);
            let decel = if remaining < 100.0 { remaining / 100.0 } else { 1.0 };

            vx += (gravity * angle.cos() + wx) * decel;
            vy += (gravity * angle.sin() + wy) * decel;

            let speed = (vx.powi(2) + vy.powi(2)).sqrt();
            if speed > max_step { let s = max_step / speed; vx *= s; vy *= s; }

            cx += vx; cy += vy;
            let jx = self.perlin.get([step as f64 * 0.15, 100.0]) * 1.2;
            let jy = self.perlin.get([step as f64 * 0.15, 200.0]) * 1.2;
            points.push(Point2D { x: cx + jx, y: cy + jy });
        }
        points
    }

    fn bezier_path(&self, start: Point2D, end: Point2D) -> Vec<Point2D> {
        let mut rng = rand::thread_rng();
        let (dx, dy) = (end.x - start.x, end.y - start.y);
        let dist = (dx * dx + dy * dy).sqrt();
        let spread = dist * 0.3;
        let cp1 = Point2D {
            x: start.x + dx * 0.3 + (rng.gen::<f64>() - 0.5) * spread,
            y: start.y + dy * 0.3 + (rng.gen::<f64>() - 0.5) * spread,
        };
        let cp2 = Point2D {
            x: start.x + dx * 0.7 + (rng.gen::<f64>() - 0.5) * spread,
            y: start.y + dy * 0.7 + (rng.gen::<f64>() - 0.5) * spread,
        };
        let steps = (dist / 3.0).max(20.0) as usize;
        let mut points = Vec::with_capacity(steps);
        for i in 0..=steps {
            let t = i as f64 / steps as f64;
            let t = if t < 0.5 { 2.0 * t * t } else { 1.0 - (-2.0 * t + 2.0).powi(2) / 2.0 };
            let u = 1.0 - t;
            let x = u.powi(3)*start.x + 3.0*u.powi(2)*t*cp1.x + 3.0*u*t.powi(2)*cp2.x + t.powi(3)*end.x;
            let y = u.powi(3)*start.y + 3.0*u.powi(2)*t*cp1.y + 3.0*u*t.powi(2)*cp2.y + t.powi(3)*end.y;
            let ji = i as f64 * 0.1 + self.noise_offset;
            let jx = self.perlin.get([ji, 50.0]) * 0.8;
            let jy = self.perlin.get([ji, 150.0]) * 0.8;
            points.push(Point2D { x: x + jx, y: y + jy });
        }
        points
    }

    fn generate_target(&self, current: Point2D, sw: i32, sh: i32) -> Point2D {
        let mut rng = rand::thread_rng();
        let intensity = self.temporal.get_intensity();
        let base = self.fingerprint.avg_speed * 0.3 * intensity;
        let dist = (base + (rng.gen::<f64>() - 0.5) * self.fingerprint.speed_variance * 0.5).max(20.0);
        let dir: usize = rng.gen_range(0..8);
        let angle = dir as f64 * std::f64::consts::PI / 4.0 + (rng.gen::<f64>() - 0.5) * 0.5;
        let m = 50.0;
        Point2D {
            x: (current.x + angle.cos() * dist).clamp(m, sw as f64 - m),
            y: (current.y + angle.sin() * dist).clamp(m, sh as f64 - m),
        }
    }

    fn score_movement(&self, points: &[Point2D]) -> AuthenticityScore {
        if points.len() < 3 { return AuthenticityScore::default(); }

        let mut speeds = Vec::new();
        for i in 1..points.len() {
            let dx = points[i].x - points[i-1].x;
            let dy = points[i].y - points[i-1].y;
            speeds.push((dx*dx + dy*dy).sqrt());
        }
        let avg = speeds.iter().sum::<f64>() / speeds.len() as f64;
        let speed_var = if avg > 0.0 {
            let v = speeds.iter().map(|s| (s - avg).powi(2)).sum::<f64>() / speeds.len() as f64;
            (v.sqrt() / avg).min(1.0) * 100.0
        } else { 0.0 };

        let mut angle_changes = Vec::new();
        for i in 2..points.len() {
            let a1 = (points[i-1].y - points[i-2].y).atan2(points[i-1].x - points[i-2].x);
            let a2 = (points[i].y - points[i-1].y).atan2(points[i].x - points[i-1].x);
            let mut d = (a2 - a1).abs();
            if d > std::f64::consts::PI { d = 2.0 * std::f64::consts::PI - d; }
            angle_changes.push(d);
        }
        let avg_a = if !angle_changes.is_empty() {
            angle_changes.iter().sum::<f64>() / angle_changes.len() as f64
        } else { 0.0 };
        let smoothness = ((1.0 - avg_a / std::f64::consts::PI) * 100.0).clamp(0.0, 100.0);

        let mut jc = 0;
        for i in 1..points.len() {
            let d = ((points[i].x-points[i-1].x).powi(2) + (points[i].y-points[i-1].y).powi(2)).sqrt();
            if d > 0.1 && d < 2.0 { jc += 1; }
        }
        let jitter = ((jc as f64 / points.len() as f64) * 200.0).clamp(0.0, 100.0);

        let mut accel = 70.0;
        if speeds.len() >= 4 {
            let q = speeds.len() / 4;
            let sa: f64 = speeds[..q].iter().sum::<f64>() / q as f64;
            let ma: f64 = speeds[q..q*3].iter().sum::<f64>() / (q * 2) as f64;
            let ea: f64 = speeds[q*3..].iter().sum::<f64>() / (speeds.len() - q*3) as f64;
            accel = if ma > sa && ma > ea { 95.0 } else if ma > sa || ma > ea { 75.0 } else { 40.0 };
        }

        let overall = (speed_var * 0.25 + smoothness * 0.25 + jitter * 0.25 + accel * 0.25).clamp(0.0, 100.0);
        AuthenticityScore { overall, speed_variance_score: speed_var, curve_smoothness: smoothness, micro_jitter_score: jitter, acceleration_pattern: accel }
    }

    fn advance_noise(&mut self) { self.noise_offset += 0.01; }
}

struct AppState {
    running: bool,
    recording: bool,
    score: AuthenticityScore,
    move_count: u64,
    status: String,
    hwnd: HWND,
    engine: AIMovementEngine,
    recorded_samples: Vec<MovementSample>,
    last_user_pos: Point2D,
}

impl AppState {
    fn new() -> Self {
        Self {
            running: false, recording: false,
            score: AuthenticityScore::default(), move_count: 0,
            status: "Ready".into(), hwnd: HWND::default(),
            engine: AIMovementEngine::new(),
            recorded_samples: Vec::new(), last_user_pos: Point2D::default(),
        }
    }
}

fn is_in_meeting() -> bool {
    use sysinfo::System;
    let mut sys = System::new();
    sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);
    let apps = ["teams", "zoom", "slack", "webex", "gotomeeting", "bluejeans", "ringcentral", "skype"];
    for (_, p) in sys.processes() {
        let name = p.name().to_string_lossy().to_lowercase();
        for app in &apps {
            if name.contains(app) && p.cpu_usage() > 5.0 { return true; }
        }
    }
    false
}

fn build_fingerprint(samples: &[MovementSample]) -> UserFingerprint {
    if samples.len() < 10 { return UserFingerprint::default(); }
    let mut speeds = Vec::new();
    let mut directions = [0.0f64; 8];

    for i in 1..samples.len() {
        let dx = samples[i].pos.x - samples[i-1].pos.x;
        let dy = samples[i].pos.y - samples[i-1].pos.y;
        let dist = (dx*dx + dy*dy).sqrt();
        let dt = samples[i].timestamp - samples[i-1].timestamp;
        if dt > 0.0 {
            let speed = dist / dt;
            if speed > 1.0 {
                speeds.push(speed);
                let angle = dy.atan2(dx);
                let idx = ((angle + std::f64::consts::PI) / (std::f64::consts::PI / 4.0)) as usize % 8;
                directions[idx] += 1.0;
            }
        }
    }

    let avg = if !speeds.is_empty() { speeds.iter().sum::<f64>() / speeds.len() as f64 } else { 450.0 };
    let var = if speeds.len() > 1 {
        (speeds.iter().map(|s| (s - avg).powi(2)).sum::<f64>() / speeds.len() as f64).sqrt()
    } else { 150.0 };

    let ds: f64 = directions.iter().sum();
    if ds > 0.0 { for d in &mut directions { *d /= ds; } }

    UserFingerprint { avg_speed: avg, speed_variance: var, preferred_directions: directions, built: true }
}

fn worker_thread(state: Arc<Mutex<AppState>>) {
    let mut rng = rand::thread_rng();
    loop {
        let (running, hwnd) = { let s = state.lock().unwrap(); (s.running, s.hwnd) };
        if !running { thread::sleep(Duration::from_millis(100)); continue; }

        if is_in_meeting() {
            { state.lock().unwrap().status = "Paused - Meeting detected".into(); }
            unsafe { let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0)); }
            thread::sleep(Duration::from_secs(30));
            continue;
        }

        let mut pt = POINT::default();
        unsafe { let _ = GetCursorPos(&mut pt); }
        let current = Point2D { x: pt.x as f64, y: pt.y as f64 };

        {
            let s = state.lock().unwrap();
            let dx = current.x - s.last_user_pos.x;
            let dy = current.y - s.last_user_pos.y;
            if (dx*dx + dy*dy).sqrt() > 30.0 {
                drop(s);
                { let mut s = state.lock().unwrap(); s.last_user_pos = current; s.status = "Paused - User active".into(); }
                unsafe { let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0)); }
                thread::sleep(Duration::from_secs(3));
                continue;
            }
        }

        let (sw, sh) = unsafe { (GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN)) };

        let (path, score) = {
            let mut s = state.lock().unwrap();
            let target = s.engine.generate_target(current, sw, sh);
            let path = if rng.gen_bool(0.6) { s.engine.wind_mouse(current, target) }
                       else { s.engine.bezier_path(current, target) };
            let score = s.engine.score_movement(&path);
            s.engine.advance_noise();
            (path, score)
        };

        for point in &path {
            if !state.lock().unwrap().running { break; }
            unsafe {
                let mut input = INPUT::default();
                input.r#type = INPUT_MOUSE;
                input.Anonymous.mi.dx = (point.x * 65535.0 / sw as f64) as i32;
                input.Anonymous.mi.dy = (point.y * 65535.0 / sh as f64) as i32;
                input.Anonymous.mi.dwFlags = MOUSEEVENTF_MOVE | MOUSEEVENTF_ABSOLUTE;
                SendInput(&[input], std::mem::size_of::<INPUT>() as i32);
            }
            thread::sleep(Duration::from_millis(rng.gen_range(3..12)));
        }

        {
            let mut s = state.lock().unwrap();
            s.score = score; s.move_count += 1;
            s.status = format!("Active - Score: {:.0}", s.score.overall);
            if let Some(last) = path.last() { s.last_user_pos = *last; }
        }
        unsafe { let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0)); }

        let intensity = state.lock().unwrap().engine.temporal.get_intensity();
        let pause = ((800.0 + rng.gen::<f64>() * 2000.0) / intensity) as u64;
        thread::sleep(Duration::from_millis(pause.clamp(500, 8000)));
    }
}

fn record_thread(state: Arc<Mutex<AppState>>) {
    let start = Instant::now();
    loop {
        if !state.lock().unwrap().recording { thread::sleep(Duration::from_millis(100)); continue; }
        let mut pt = POINT::default();
        unsafe { let _ = GetCursorPos(&mut pt); }
        let sample = MovementSample { pos: Point2D { x: pt.x as f64, y: pt.y as f64 }, timestamp: start.elapsed().as_secs_f64() };
        let hwnd;
        {
            let mut s = state.lock().unwrap();
            s.recorded_samples.push(sample);
            s.status = format!("Recording... {} samples", s.recorded_samples.len());
            hwnd = s.hwnd;
        }
        unsafe { let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0)); }
        thread::sleep(Duration::from_millis(16));
    }
}

static mut APP_STATE: Option<Arc<Mutex<AppState>>> = None;

fn wide(s: &str) -> Vec<u16> { s.encode_utf16().chain(std::iter::once(0)).collect() }

unsafe fn mkfont(size: i32, bold: bool) -> HFONT {
    CreateFontW(size, 0, 0, 0, if bold { 700 } else { 400 }, 0, 0, 0, 0, 0, 0, CLEARTYPE_QUALITY.0 as u32, 0, w!("Segoe UI"))
}

unsafe fn set_text(hwnd: HWND, id: i32, text: &str) {
    let child = GetDlgItem(hwnd, id);
    if child.0 != 0 { let _ = SetWindowTextW(child, &HSTRING::from(text)); }
}

unsafe extern "system" fn wndproc(hwnd: HWND, msg: u32, wp: WPARAM, lp: LPARAM) -> LRESULT {
    match msg {
        WM_CREATE => {
            let f = mkfont(16, false);
            let bf = mkfont(20, true);
            let tf = mkfont(28, true);

            let mk = |text: &str, style: WINDOW_STYLE, x, y, w, h, id: Option<i32>, font: HFONT, class: &str| {
                let hmenu = id.map(|i| HMENU(i as _)).unwrap_or(HMENU(0));
                let h = CreateWindowExW(WINDOW_EX_STYLE(0), &HSTRING::from(class), &HSTRING::from(text), style, x, y, w, h, hwnd, hmenu, HINSTANCE::default(), None);
                SendMessageW(h, WM_SETFONT, WPARAM(font.0 as usize), LPARAM(1));
            };

            let ss = WS_CHILD | WS_VISIBLE | WINDOW_STYLE(MY_SS_CENTER);
            let sl = WS_CHILD | WS_VISIBLE;
            let bs = WS_CHILD | WS_VISIBLE | WINDOW_STYLE(BS_PUSHBUTTON as u32);

            mk("WiggleMe AI", ss, 20, 15, 440, 40, None, tf, "STATIC");
            mk("AI-Powered Human-Kinematic Mouse Simulator", ss, 20, 50, 440, 20, None, f, "STATIC");
            mk("Authenticity Score: --", sl, 30, 90, 420, 30, Some(IDC_STATIC_SCORE), bf, "STATIC");
            mk("Detection Risk: --", sl, 30, 125, 420, 25, Some(IDC_STATIC_RISK), f, "STATIC");
            mk("Movements: 0", sl, 30, 155, 420, 25, Some(IDC_STATIC_MOVES), f, "STATIC");
            mk("Status: Ready", sl, 30, 185, 420, 25, Some(IDC_STATIC_STATUS), f, "STATIC");
            mk("START", bs, 30, 230, 200, 45, Some(IDC_BTN_START), bf, "BUTTON");
            mk("LEARN MY MOVEMENT", bs, 250, 230, 200, 45, Some(IDC_BTN_LEARN), f, "BUTTON");

            if let Some(st) = &APP_STATE { st.lock().unwrap().hwnd = hwnd; }
            LRESULT(0)
        }

        WM_COMMAND => {
            let id = (wp.0 & 0xFFFF) as i32;
            let notify = ((wp.0 >> 16) & 0xFFFF) as u32;
            if notify == BN_CLICKED as u32 {
                if let Some(state) = &APP_STATE {
                    match id {
                        IDC_BTN_START => {
                            let mut s = state.lock().unwrap();
                            s.running = !s.running;
                            let txt = if s.running { "STOP" } else { "START" };
                            let btn = GetDlgItem(hwnd, IDC_BTN_START);
                            let _ = SetWindowTextW(btn, &HSTRING::from(txt));
                            s.status = if s.running { "Starting...".into() } else { "Stopped".into() };
                            drop(s);
                            let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0));
                        }
                        IDC_BTN_LEARN => {
                            let mut s = state.lock().unwrap();
                            let btn = GetDlgItem(hwnd, IDC_BTN_LEARN);
                            if s.recording {
                                s.recording = false;
                                let fp = build_fingerprint(&s.recorded_samples);
                                if fp.built {
                                    let n = s.recorded_samples.len();
                                    s.engine.set_fingerprint(fp);
                                    s.status = format!("Fingerprint built from {} samples!", n);
                                } else {
                                    s.status = "Not enough data, try again".into();
                                }
                                let _ = SetWindowTextW(btn, w!("LEARN MY MOVEMENT"));
                            } else {
                                s.recording = true;
                                s.recorded_samples.clear();
                                s.status = "Recording... Move your mouse naturally".into();
                                let _ = SetWindowTextW(btn, w!("STOP LEARNING"));
                            }
                            drop(s);
                            let _ = PostMessageW(hwnd, WM_APP_UPDATE, WPARAM(0), LPARAM(0));
                        }
                        _ => {}
                    }
                }
            }
            LRESULT(0)
        }

        WM_APP_UPDATE => {
            if let Some(state) = &APP_STATE {
                let s = state.lock().unwrap();
                set_text(hwnd, IDC_STATIC_SCORE, &format!("Authenticity Score: {:.0}/100", s.score.overall));
                set_text(hwnd, IDC_STATIC_RISK, &format!("Detection Risk: {}", s.score.risk_level()));
                set_text(hwnd, IDC_STATIC_MOVES, &format!("Movements: {}", s.move_count));
                set_text(hwnd, IDC_STATIC_STATUS, &format!("Status: {}", s.status));
            }
            InvalidateRect(hwnd, None, false);
            LRESULT(0)
        }

        WM_CTLCOLORSTATIC => {
            let hdc = HDC(wp.0 as isize);
            SetTextColor(hdc, COLORREF(TEXT_COLOR));
            SetBkColor(hdc, COLORREF(BG_COLOR));
            return LRESULT(CreateSolidBrush(COLORREF(BG_COLOR)).0 as isize);
        }

        WM_ERASEBKGND => {
            let hdc = HDC(wp.0 as isize);
            let mut rc = RECT::default();
            let _ = GetClientRect(hwnd, &mut rc);
            let brush = CreateSolidBrush(COLORREF(BG_COLOR));
            FillRect(hdc, &rc, brush);
            let _ = DeleteObject(brush);

            if let Some(state) = &APP_STATE {
                let s = state.lock().unwrap();
                let (bx, by, bw, bh) = (30, 280, 420, 20);
                let bg_r = RECT { left: bx, top: by, right: bx+bw, bottom: by+bh };
                let db = CreateSolidBrush(COLORREF(0x00302020));
                FillRect(hdc, &bg_r, db); let _ = DeleteObject(db);

                let fw = (bw as f64 * s.score.overall / 100.0) as i32;
                let color = if s.score.overall >= 80.0 { GREEN_COLOR }
                            else if s.score.overall >= 50.0 { YELLOW_COLOR }
                            else { RED_COLOR };
                let fr = RECT { left: bx, top: by, right: bx+fw, bottom: by+bh };
                let fb = CreateSolidBrush(COLORREF(color));
                FillRect(hdc, &fr, fb); let _ = DeleteObject(fb);
            }
            return LRESULT(1);
        }

        WM_CLOSE => { let _ = DestroyWindow(hwnd); LRESULT(0) }
        WM_DESTROY => { PostQuitMessage(0); LRESULT(0) }
        _ => DefWindowProcW(hwnd, msg, wp, lp),
    }
}

fn main() {
    unsafe {
        let icc = INITCOMMONCONTROLSEX { dwSize: std::mem::size_of::<INITCOMMONCONTROLSEX>() as u32, dwICC: ICC_STANDARD_CLASSES };
        InitCommonControlsEx(&icc);

        let instance = GetModuleHandleW(None).unwrap();
        let cn = wide("WiggleMeAI");

        let wc = WNDCLASSEXW {
            cbSize: std::mem::size_of::<WNDCLASSEXW>() as u32,
            style: CS_HREDRAW | CS_VREDRAW,
            lpfnWndProc: Some(wndproc),
            hInstance: instance.into(),
            hCursor: LoadCursorW(None, IDC_ARROW).unwrap(),
            hbrBackground: CreateSolidBrush(COLORREF(BG_COLOR)),
            lpszClassName: PCWSTR(cn.as_ptr()),
            ..Default::default()
        };
        RegisterClassExW(&wc);

        let state = Arc::new(Mutex::new(AppState::new()));
        APP_STATE = Some(state.clone());

        let hwnd = CreateWindowExW(
            WINDOW_EX_STYLE(0), PCWSTR(cn.as_ptr()),
            w!("WiggleMe AI - Human-Kinematic Mouse Simulator"),
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
            CW_USEDEFAULT, CW_USEDEFAULT, 500, 340,
            HWND::default(), HMENU::default(), instance, None,
        );

        let _ = ShowWindow(hwnd, SW_SHOW);
        UpdateWindow(hwnd);

        let sw = state.clone();
        thread::spawn(move || worker_thread(sw));
        let sr = state.clone();
        thread::spawn(move || record_thread(sr));

        let mut msg = MSG::default();
        while GetMessageW(&mut msg, HWND::default(), 0, 0).into() {
            let _ = TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }
}
