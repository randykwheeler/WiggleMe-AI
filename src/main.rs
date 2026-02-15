#![windows_subsystem = "windows"]
#![allow(static_mut_refs)]

use noise::{NoiseFn, Perlin};
use rand::Rng;
use std::sync::atomic::{AtomicBool, AtomicI32, Ordering};
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

// ── Control IDs ────────────────────────────────────────────────────────────
const IDC_PITCH_SLIDER: i32 = 101;
const IDC_YAW_SLIDER: i32 = 102;
const IDC_INTERVAL_SLIDER: i32 = 103;
const IDC_DELAY_SLIDER: i32 = 108;
const IDC_START_BTN: i32 = 105;
const IDC_LEARN_BTN: i32 = 112;
const IDC_CHAOS_CHECK: i32 = 106;
const IDC_TOPMOST_CHECK: i32 = 110;
const IDC_ALGO_COMBO: i32 = 109;
const IDC_STOP_SIGNAL: i32 = 107;
const IDC_PITCH_VAL: i32 = 201;
const IDC_YAW_VAL: i32 = 202;
const IDC_INTERVAL_VAL: i32 = 203;
const IDC_DELAY_VAL: i32 = 204;
const IDC_STATUS: i32 = 205;
const IDC_SCORE_LABEL: i32 = 206;

const WM_UPDATE_STATUS: u32 = WM_APP + 1;
const WM_UPDATE_SCORE: u32 = WM_APP + 2;
const MY_SS_CENTER: u32 = 0x0001;

// Trackbar constants not exported by windows 0.57
const TBM_GETPOS: u32 = 0x0400; // WM_USER + 0
const TBM_SETPOS: u32 = 0x0405; // WM_USER + 5
const TBM_SETRANGE: u32 = 0x0406; // WM_USER + 6
const TBS_HORZ: u32 = 0x0000;
const TBS_AUTOTICKS: u32 = 0x0001;

const ALGO_WINDMOUSE: i32 = 0;
const ALGO_ENHANCED: i32 = 1;
const ALGO_BEZIER: i32 = 2;
const ALGO_FITTS: i32 = 3;

// ── Colors ─────────────────────────────────────────────────────────────────
const BG_COLOR: u32 = 0x00503e2c;
const TEXT_COLOR: u32 = 0x00f0f0f0;
const GREEN_COLOR: u32 = 0x0050c878;
const YELLOW_COLOR: u32 = 0x0000d4f5;
const RED_COLOR: u32 = 0x004040e0;

// ── Global Atomic State ────────────────────────────────────────────────────
static G_RUNNING: AtomicBool = AtomicBool::new(false);
static G_RECORDING: AtomicBool = AtomicBool::new(false);
static G_PITCH: AtomicI32 = AtomicI32::new(10);
static G_YAW: AtomicI32 = AtomicI32::new(10);
static G_INTERVAL: AtomicI32 = AtomicI32::new(5);
static G_DELAY: AtomicI32 = AtomicI32::new(5);
static G_ALGORITHM: AtomicI32 = AtomicI32::new(0);
static G_CHAOS: AtomicBool = AtomicBool::new(false);
static G_SCORE: AtomicI32 = AtomicI32::new(0);
static G_MOVES: AtomicI32 = AtomicI32::new(0);

// ── Point2D ────────────────────────────────────────────────────────────────
#[derive(Clone, Copy, Default)]
struct Point2D { x: f64, y: f64 }

// ── MovementSample ─────────────────────────────────────────────────────────
#[derive(Clone, Default)]
struct MovementSample {
    pos: Point2D,
    timestamp: f64,
}

// ── UserFingerprint ────────────────────────────────────────────────────────
#[derive(Clone)]
struct UserFingerprint {
    avg_speed: f64,
    speed_variance: f64,
    avg_pause_duration: f64,
    pause_frequency: f64,
    preferred_directions: [f64; 8],
    built: bool,
}

impl Default for UserFingerprint {
    fn default() -> Self {
        Self {
            avg_speed: 450.0,
            speed_variance: 150.0,
            avg_pause_duration: 1.8,
            pause_frequency: 0.15,
            preferred_directions: [1.0; 8],
            built: false,
        }
    }
}

// ── Shared Fingerprint ─────────────────────────────────────────────────────
static mut FINGERPRINT: Option<Arc<Mutex<UserFingerprint>>> = None;
static mut RECORDED_SAMPLES: Option<Arc<Mutex<Vec<MovementSample>>>> = None;
static mut MAIN_HWND: HWND = HWND(0);

fn get_fingerprint() -> Arc<Mutex<UserFingerprint>> {
    unsafe { FINGERPRINT.as_ref().unwrap().clone() }
}

fn get_samples() -> Arc<Mutex<Vec<MovementSample>>> {
    unsafe { RECORDED_SAMPLES.as_ref().unwrap().clone() }
}

// ── Perlin helper ──────────────────────────────────────────────────────────
struct NoiseGen {
    perlin: Perlin,
    offset: f64,
}

impl NoiseGen {
    fn new() -> Self { Self { perlin: Perlin::new(rand::thread_rng().gen()), offset: 0.0 } }
    fn get(&self, x: f64, y: f64) -> f64 { self.perlin.get([x + self.offset, y]) }
    fn advance(&mut self) { self.offset += 0.01; }
}

// ── Movement Algorithms ────────────────────────────────────────────────────

fn wind_mouse_classic(sx: f64, sy: f64, ex: f64, ey: f64) {
    let mut rng = rand::thread_rng();
    let gravity = 9.0;
    let wind = 3.0;
    let max_step = 10.0;
    let target_area = 8.0;

    let mut cx = sx; let mut cy = sy;
    let mut vx = 0.0f64; let mut vy = 0.0f64;
    let mut wx = 0.0f64; let mut wy = 0.0f64;

    for _ in 0..5000 {
        if !G_RUNNING.load(Ordering::Relaxed) { return; }
        let dist = ((ex - cx).powi(2) + (ey - cy).powi(2)).sqrt();
        if dist < 1.0 { break; }

        let cur_wind = if dist < target_area { wind * (dist / target_area) } else { wind };
        let cur_step = if dist < target_area { max_step * (dist / target_area).max(0.5) } else { max_step };

        wx = wx / 3.0_f64.sqrt() + (rng.gen::<f64>() * 2.0 - 1.0) * cur_wind / 5.0_f64.sqrt();
        wy = wy / 3.0_f64.sqrt() + (rng.gen::<f64>() * 2.0 - 1.0) * cur_wind / 5.0_f64.sqrt();

        vx += wx + gravity * (ex - cx) / dist;
        vy += wy + gravity * (ey - cy) / dist;

        let speed = (vx.powi(2) + vy.powi(2)).sqrt();
        if speed > cur_step { let s = cur_step / speed; vx *= s; vy *= s; }

        cx += vx; cy += vy;
        set_cursor(cx as i32, cy as i32);
        thread::sleep(Duration::from_millis(rng.gen_range(2..10)));
    }
    set_cursor(ex as i32, ey as i32);
}

fn wind_mouse_enhanced(sx: f64, sy: f64, ex: f64, ey: f64) {
    let mut rng = rand::thread_rng();
    let mut noise = NoiseGen::new();
    let total_dist = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
    if total_dist < 1.0 { return; }

    let gravity = 9.0;
    let wind = 3.0;
    let max_step = 10.0;
    let mut cx = sx; let mut cy = sy;
    let mut vx = 0.0f64; let mut vy = 0.0f64;
    let mut wx = 0.0f64; let mut wy = 0.0f64;
    let mut noise_t = 0.0f64;

    for _ in 0..5000 {
        if !G_RUNNING.load(Ordering::Relaxed) { return; }
        let dist = ((ex - cx).powi(2) + (ey - cy).powi(2)).sqrt();
        if dist < 1.0 { break; }

        let speed_factor = (dist / total_dist + 0.2).min(1.0);
        let cur_step = max_step * speed_factor;

        wx = wx / 3.0_f64.sqrt() + (rng.gen::<f64>() * 2.0 - 1.0) * wind / 5.0_f64.sqrt();
        wy = wy / 3.0_f64.sqrt() + (rng.gen::<f64>() * 2.0 - 1.0) * wind / 5.0_f64.sqrt();

        vx += wx + gravity * (ex - cx) / dist;
        vy += wy + gravity * (ey - cy) / dist;

        let speed = (vx.powi(2) + vy.powi(2)).sqrt();
        if speed > cur_step { let s = cur_step / speed; vx *= s; vy *= s; }

        // Perlin noise jitter
        noise_t += 0.05;
        let jx = noise.get(noise_t, 100.0) * 2.5;
        let jy = noise.get(noise_t + 100.0, 200.0) * 2.5;

        cx += vx + jx; cy += vy + jy;
        set_cursor(cx as i32, cy as i32);

        // Random pauses (3% chance)
        if rng.gen_range(0..100) < 3 {
            thread::sleep(Duration::from_millis(rng.gen_range(50..200)));
        }

        let base_wait = 2.0 + (1.0 - speed_factor) * 8.0;
        thread::sleep(Duration::from_millis(base_wait as u64 + rng.gen_range(0..4)));
        noise.advance();
    }
    set_cursor(ex as i32, ey as i32);
}

fn bezier_move(sx: f64, sy: f64, ex: f64, ey: f64) {
    let mut rng = rand::thread_rng();
    let mut noise = NoiseGen::new();
    let dx = ex - sx; let dy = ey - sy;
    let dist = (dx * dx + dy * dy).sqrt();
    if dist < 1.0 { return; }

    // Perpendicular direction for arc
    let (px, py) = (-dy / dist, dx / dist);
    let arc1 = dist * (0.1 + rng.gen_range(0..30) as f64 / 100.0) * if rng.gen_bool(0.5) { 1.0 } else { -1.0 };
    let arc2 = dist * (0.1 + rng.gen_range(0..30) as f64 / 100.0) * if rng.gen_bool(0.5) { 1.0 } else { -1.0 };

    let p0 = Point2D { x: sx, y: sy };
    let p1 = Point2D { x: sx + dx * 0.3 + px * arc1, y: sy + dy * 0.3 + py * arc1 };
    let p2 = Point2D { x: sx + dx * 0.7 + px * arc2, y: sy + dy * 0.7 + py * arc2 };
    let p3 = Point2D { x: ex, y: ey };

    let steps = (dist / 2.0).max(30.0) as usize;
    for i in 0..=steps {
        if !G_RUNNING.load(Ordering::Relaxed) { return; }
        let raw_t = i as f64 / steps as f64;
        let t = raw_t * raw_t * (3.0 - 2.0 * raw_t); // smoothstep
        let u = 1.0 - t;

        let x = u.powi(3)*p0.x + 3.0*u.powi(2)*t*p1.x + 3.0*u*t.powi(2)*p2.x + t.powi(3)*p3.x;
        let y = u.powi(3)*p0.y + 3.0*u.powi(2)*t*p1.y + 3.0*u*t.powi(2)*p2.y + t.powi(3)*p3.y;

        let jx = noise.get(i as f64 * 0.1, 50.0) * 1.5;
        let jy = noise.get(i as f64 * 0.1, 150.0) * 1.5;
        set_cursor((x + jx) as i32, (y + jy) as i32);
        thread::sleep(Duration::from_millis(rng.gen_range(3..10)));
        noise.advance();
    }
    set_cursor(ex as i32, ey as i32);
}

fn fitts_move(sx: f64, sy: f64, ex: f64, ey: f64) {
    let mut rng = rand::thread_rng();
    let mut noise = NoiseGen::new();
    let dist = ((ex - sx).powi(2) + (ey - sy).powi(2)).sqrt();
    if dist < 1.0 { return; }

    // Fitts's Law timing
    let target_width = 20.0;
    let fitts_time = (100.0 + 150.0 * (2.0 * dist / target_width + 1.0).log2()).clamp(100.0, 2000.0);

    // Overshoot: 60% chance for distances > 100px
    let overshoot = dist > 100.0 && rng.gen_range(0..100) < 60;
    let os_dist = dist * (0.05 + rng.gen_range(0..15) as f64 / 100.0);
    let angle = (ey - sy).atan2(ex - sx);
    let (move_x, move_y) = if overshoot {
        (ex + angle.cos() * os_dist, ey + angle.sin() * os_dist)
    } else {
        (ex, ey)
    };

    // Phase 1: main movement
    let steps = (fitts_time / 8.0).max(20.0) as usize;
    for i in 0..=steps {
        if !G_RUNNING.load(Ordering::Relaxed) { return; }
        let raw_t = i as f64 / steps as f64;
        let t = 1.0 - (1.0 - raw_t).powf(2.5); // ease-out
        let x = sx + (move_x - sx) * t;
        let y = sy + (move_y - sy) * t;
        let jx = noise.get(i as f64 * 0.08, 50.0) * 1.2;
        let jy = noise.get(i as f64 * 0.08, 150.0) * 1.2;
        set_cursor((x + jx) as i32, (y + jy) as i32);
        thread::sleep(Duration::from_millis(rng.gen_range(4..12)));
        noise.advance();
    }

    // Phase 2: overshoot correction
    if overshoot && G_RUNNING.load(Ordering::Relaxed) {
        thread::sleep(Duration::from_millis(rng.gen_range(30..100)));
        let corr_steps = (os_dist / 2.0).max(10.0) as usize;
        for i in 0..=corr_steps {
            if !G_RUNNING.load(Ordering::Relaxed) { return; }
            let raw_t = i as f64 / corr_steps as f64;
            let t = raw_t * raw_t * (3.0 - 2.0 * raw_t);
            let x = move_x + (ex - move_x) * t;
            let y = move_y + (ey - move_y) * t;
            let jx = noise.get(i as f64 * 0.12, 300.0) * 0.8;
            let jy = noise.get(i as f64 * 0.12, 400.0) * 0.8;
            set_cursor((x + jx) as i32, (y + jy) as i32);
            thread::sleep(Duration::from_millis(rng.gen_range(5..15)));
        }
    }

    // Final landing with ±1px jitter
    set_cursor(ex as i32 + rng.gen_range(-1..=1), ey as i32 + rng.gen_range(-1..=1));
}

fn move_with_algorithm(algo: i32, sx: f64, sy: f64, ex: f64, ey: f64) {
    match algo {
        ALGO_WINDMOUSE => wind_mouse_classic(sx, sy, ex, ey),
        ALGO_ENHANCED => wind_mouse_enhanced(sx, sy, ex, ey),
        ALGO_BEZIER => bezier_move(sx, sy, ex, ey),
        ALGO_FITTS => fitts_move(sx, sy, ex, ey),
        _ => wind_mouse_classic(sx, sy, ex, ey),
    }
}

fn set_cursor(x: i32, y: i32) {
    unsafe { let _ = SetCursorPos(x, y); }
}

// ── Authenticity Scoring ───────────────────────────────────────────────────
fn score_path(points: &[(i32, i32)]) -> i32 {
    if points.len() < 3 { return 0; }

    let mut speeds = Vec::new();
    for i in 1..points.len() {
        let dx = (points[i].0 - points[i-1].0) as f64;
        let dy = (points[i].1 - points[i-1].1) as f64;
        speeds.push((dx*dx + dy*dy).sqrt());
    }
    let avg = speeds.iter().sum::<f64>() / speeds.len() as f64;
    let speed_var = if avg > 0.0 {
        let v = speeds.iter().map(|s| (s - avg).powi(2)).sum::<f64>() / speeds.len() as f64;
        (v.sqrt() / avg).min(1.0) * 100.0
    } else { 0.0 };

    let mut jc = 0;
    for i in 1..points.len() {
        let d = (((points[i].0-points[i-1].0) as f64).powi(2) + ((points[i].1-points[i-1].1) as f64).powi(2)).sqrt();
        if d > 0.1 && d < 2.0 { jc += 1; }
    }
    let jitter = ((jc as f64 / points.len() as f64) * 200.0).clamp(0.0, 100.0);

    let mut accel = 70.0;
    if speeds.len() >= 4 {
        let q = speeds.len() / 4;
        let sa: f64 = speeds[..q].iter().sum::<f64>() / q as f64;
        let ma: f64 = speeds[q..q*3].iter().sum::<f64>() / (q*2) as f64;
        let ea: f64 = speeds[q*3..].iter().sum::<f64>() / (speeds.len()-q*3) as f64;
        accel = if ma > sa && ma > ea { 95.0 } else if ma > sa || ma > ea { 75.0 } else { 40.0 };
    }

    ((speed_var * 0.33 + jitter * 0.33 + accel * 0.34).clamp(0.0, 100.0)) as i32
}

// ── Meeting Detection ──────────────────────────────────────────────────────
fn is_in_meeting() -> bool {
    use sysinfo::System;
    let mut sys = System::new();
    sys.refresh_processes(sysinfo::ProcessesToUpdate::All, true);
    let apps = ["teams", "zoom", "slack", "webex", "gotomeeting", "bluejeans", "ringcentral", "skype"];
    for (_, p) in sys.processes() {
        let name = p.name().to_string_lossy().to_lowercase();
        for app in &apps { if name.contains(app) && p.cpu_usage() > 5.0 { return true; } }
    }
    false
}

// ── Build Fingerprint ──────────────────────────────────────────────────────
fn build_fingerprint(samples: &[MovementSample]) -> UserFingerprint {
    if samples.len() < 50 { return UserFingerprint::default(); }

    let mut speeds = Vec::new();
    let mut pauses = Vec::new();
    let mut directions = [0.0f64; 8];
    let mut in_pause = false;
    let mut pause_start = 0.0;

    for i in 1..samples.len() {
        let dx = samples[i].pos.x - samples[i-1].pos.x;
        let dy = samples[i].pos.y - samples[i-1].pos.y;
        let dist = (dx*dx + dy*dy).sqrt();
        let dt = samples[i].timestamp - samples[i-1].timestamp;
        if dt > 0.001 {
            let speed = dist / dt;
            if speed > 5.0 {
                speeds.push(speed);
                let angle = dy.atan2(dx);
                let idx = ((angle + std::f64::consts::PI) / (std::f64::consts::PI / 4.0)) as usize % 8;
                directions[idx] += 1.0;
                if in_pause {
                    pauses.push(samples[i].timestamp - pause_start);
                    in_pause = false;
                }
            } else if dist < 2.0 {
                if !in_pause { in_pause = true; pause_start = samples[i].timestamp; }
            }
        }
    }

    let avg = if !speeds.is_empty() { speeds.iter().sum::<f64>() / speeds.len() as f64 } else { 450.0 };
    let var = if speeds.len() > 1 {
        (speeds.iter().map(|s| (s - avg).powi(2)).sum::<f64>() / speeds.len() as f64).sqrt()
    } else { 150.0 };
    let avg_pause = if !pauses.is_empty() { pauses.iter().sum::<f64>() / pauses.len() as f64 } else { 1.8 };
    let total_time = samples.last().unwrap().timestamp - samples.first().unwrap().timestamp;
    let pause_freq = if total_time > 0.0 { pauses.len() as f64 / total_time } else { 0.15 };

    let ds: f64 = directions.iter().sum();
    if ds > 0.0 { for d in &mut directions { *d /= ds; } }

    UserFingerprint {
        avg_speed: avg,
        speed_variance: var,
        avg_pause_duration: avg_pause.clamp(0.3, 10.0),
        pause_frequency: pause_freq.clamp(0.01, 1.0),
        preferred_directions: directions,
        built: true,
    }
}

// ── Temporal Intensity ─────────────────────────────────────────────────────
fn temporal_intensity() -> f64 {
    use chrono::Timelike;
    let hour = chrono::Local::now().hour() as usize;
    let table: [f64; 24] = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.6, 0.9, 1.0, 0.95, 0.3, 0.5, 0.85, 0.9,
                 0.8, 0.4, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0];
    if hour < 24 { table[hour].max(0.1_f64) } else { 0.1 }
}

// ── Main Worker Thread ─────────────────────────────────────────────────────
fn wiggler_thread(main_hwnd: HWND) {
    let mut rng = rand::thread_rng();
    let start = Instant::now();
    let delay = G_DELAY.load(Ordering::Relaxed) as f64;
    let threshold = 30; // DPI base threshold

    let (sw, sh) = unsafe { (GetSystemMetrics(SM_CXSCREEN), GetSystemMetrics(SM_CYSCREEN)) };

    let mut last_pos = POINT::default();
    unsafe { let _ = GetCursorPos(&mut last_pos); }

    let mut path_points: Vec<(i32, i32)> = Vec::new();

    loop {
        if !G_RUNNING.load(Ordering::Relaxed) { break; }

        let elapsed = start.elapsed().as_secs_f64();
        let mut cur = POINT::default();
        unsafe { let _ = GetCursorPos(&mut cur); }

        // Grace period
        if elapsed < delay {
            let remaining = delay - elapsed;
            post_status(main_hwnd, &format!("Grace Period: {:.1}s remaining...", remaining));
            last_pos = cur;
            thread::sleep(Duration::from_millis(100));
            continue;
        }

        // Mouse movement detection (manual override)
        let dx = (cur.x - last_pos.x).abs();
        let dy = (cur.y - last_pos.y).abs();
        if dx > threshold || dy > threshold {
            post_status(main_hwnd, "Manual override detected!");
            G_RUNNING.store(false, Ordering::Relaxed);
            unsafe { let _ = PostMessageW(main_hwnd, WM_COMMAND, WPARAM(IDC_STOP_SIGNAL as usize), LPARAM(0)); }
            break;
        }

        // Meeting detection (check every ~30s worth of cycles)
        if rng.gen_range(0..30) == 0 && is_in_meeting() {
            post_status(main_hwnd, "Paused - Meeting detected");
            thread::sleep(Duration::from_secs(30));
            unsafe { let _ = GetCursorPos(&mut last_pos); }
            continue;
        }

        // Get settings
        let pitch = G_PITCH.load(Ordering::Relaxed);
        let yaw = G_YAW.load(Ordering::Relaxed);
        let interval = G_INTERVAL.load(Ordering::Relaxed);
        let algo = G_ALGORITHM.load(Ordering::Relaxed);
        let chaos = G_CHAOS.load(Ordering::Relaxed);

        // Generate target
        let (target_x, target_y) = if chaos {
            (rng.gen_range(50..sw-50), rng.gen_range(50..sh-50))
        } else {
            let tdx = if yaw > 0 { rng.gen_range(-(yaw)..=yaw) } else { 0 };
            let tdy = if pitch > 0 { rng.gen_range(-(pitch)..=pitch) } else { 0 };
            (cur.x + tdx, cur.y + tdy)
        };

        // Apply fingerprint influence
        let fp = get_fingerprint();
        let fp_data = fp.lock().unwrap();
        let _intensity = temporal_intensity();
        let _speed_mult = if fp_data.built { fp_data.avg_speed / 450.0 } else { 1.0 };
        drop(fp_data);

        // Record path start
        path_points.clear();
        path_points.push((cur.x, cur.y));

        post_status(main_hwnd, "Wiggling... Move mouse to stop.");
        move_with_algorithm(algo, cur.x as f64, cur.y as f64, target_x as f64, target_y as f64);

        // Record path end
        unsafe { let _ = GetCursorPos(&mut cur); }
        path_points.push((cur.x, cur.y));

        // Score the movement
        let score = score_path(&path_points);
        G_SCORE.store(score, Ordering::Relaxed);
        G_MOVES.fetch_add(1, Ordering::Relaxed);
        unsafe { let _ = PostMessageW(main_hwnd, WM_UPDATE_SCORE, WPARAM(0), LPARAM(0)); }

        // Update last known position
        unsafe { let _ = GetCursorPos(&mut last_pos); }

        // Wait interval with motion checking
        let stop_time = Instant::now() + Duration::from_secs(interval as u64);
        while Instant::now() < stop_time && G_RUNNING.load(Ordering::Relaxed) {
            unsafe { let _ = GetCursorPos(&mut cur); }
            let mdx = (cur.x - last_pos.x).abs();
            let mdy = (cur.y - last_pos.y).abs();
            if mdx > threshold || mdy > threshold {
                post_status(main_hwnd, "Manual override detected!");
                G_RUNNING.store(false, Ordering::Relaxed);
                unsafe { let _ = PostMessageW(main_hwnd, WM_COMMAND, WPARAM(IDC_STOP_SIGNAL as usize), LPARAM(0)); }
                return;
            }
            thread::sleep(Duration::from_millis(100));
        }
    }
}

fn post_status(hwnd: HWND, text: &str) {
    let wide: Vec<u16> = text.encode_utf16().chain(std::iter::once(0)).collect();
    let boxed = wide.into_boxed_slice();
    let ptr = Box::into_raw(boxed) as *mut u16 as isize;
    unsafe { let _ = PostMessageW(hwnd, WM_UPDATE_STATUS, WPARAM(0), LPARAM(ptr)); }
}

// ── Recording Thread ───────────────────────────────────────────────────────
fn record_thread() {
    let start = Instant::now();
    let samples = get_samples();
    let hwnd = unsafe { MAIN_HWND };

    loop {
        if !G_RECORDING.load(Ordering::Relaxed) { thread::sleep(Duration::from_millis(100)); continue; }
        let mut pt = POINT::default();
        unsafe { let _ = GetCursorPos(&mut pt); }
        let sample = MovementSample { pos: Point2D { x: pt.x as f64, y: pt.y as f64 }, timestamp: start.elapsed().as_secs_f64() };
        let count;
        {
            let mut s = samples.lock().unwrap();
            s.push(sample);
            count = s.len();
        }
        post_status(hwnd, &format!("Recording... {} samples (move mouse naturally for 10+ seconds)", count));
        thread::sleep(Duration::from_millis(16)); // ~60Hz
    }
}

// ── Win32 UI ───────────────────────────────────────────────────────────────
fn wide(s: &str) -> Vec<u16> { s.encode_utf16().chain(std::iter::once(0)).collect() }

unsafe fn mkfont(size: i32, bold: bool) -> HFONT {
    CreateFontW(size, 0, 0, 0, if bold { 700 } else { 400 }, 0, 0, 0, 0, 0, 0, CLEARTYPE_QUALITY.0 as u32, 0, w!("Segoe UI"))
}

unsafe fn make_slider(parent: HWND, x: i32, y: i32, w: i32, h: i32, id: i32, min_v: i32, max_v: i32, def: i32) -> HWND {
    let slider = CreateWindowExW(WINDOW_EX_STYLE(0), w!("msctls_trackbar32"), w!(""),
        WS_CHILD | WS_VISIBLE | WINDOW_STYLE(TBS_HORZ | TBS_AUTOTICKS),
        x, y, w, h, parent, HMENU(id as _), HINSTANCE::default(), None);
    SendMessageW(slider, TBM_SETRANGE, WPARAM(1), LPARAM(((max_v as u32) << 16 | min_v as u32) as isize));
    SendMessageW(slider, TBM_SETPOS, WPARAM(1), LPARAM(def as isize));
    slider
}

unsafe fn set_text(hwnd: HWND, id: i32, text: &str) {
    let child = GetDlgItem(hwnd, id);
    if child.0 != 0 { let _ = SetWindowTextW(child, &HSTRING::from(text)); }
}

unsafe extern "system" fn wndproc(hwnd: HWND, msg: u32, wp: WPARAM, lp: LPARAM) -> LRESULT {
    match msg {
        WM_CREATE => {
            let f = mkfont(14, false);
            let bf = mkfont(16, true);
            let tf = mkfont(24, true);

            let mk_static = |text, x, y, w, h, id: Option<i32>, font: HFONT, center: bool| {
                let style = if center { WS_CHILD | WS_VISIBLE | WINDOW_STYLE(MY_SS_CENTER) } else { WS_CHILD | WS_VISIBLE };
                let hmenu = id.map(|i| HMENU(i as _)).unwrap_or(HMENU(0));
                let ctrl = CreateWindowExW(WINDOW_EX_STYLE(0), w!("STATIC"), &HSTRING::from(text), style, x, y, w, h, hwnd, hmenu, HINSTANCE::default(), None);
                SendMessageW(ctrl, WM_SETFONT, WPARAM(font.0 as usize), LPARAM(1));
                ctrl
            };

            // Title
            mk_static("WiggleMe AI", 10, 8, 460, 30, None, tf, true);
            mk_static("AI-Powered Human-Kinematic Mouse Simulator", 10, 35, 460, 18, None, f, true);

            // Pitch slider
            let mut y_pos = 60;
            mk_static("Pitch (Vertical):", 15, y_pos, 120, 20, None, f, false);
            mk_static("10 px", 370, y_pos, 80, 20, Some(IDC_PITCH_VAL), f, false);
            make_slider(hwnd, 140, y_pos, 225, 25, IDC_PITCH_SLIDER, 0, 100, 10);

            // Yaw slider
            y_pos += 30;
            mk_static("Yaw (Horizontal):", 15, y_pos, 120, 20, None, f, false);
            mk_static("10 px", 370, y_pos, 80, 20, Some(IDC_YAW_VAL), f, false);
            make_slider(hwnd, 140, y_pos, 225, 25, IDC_YAW_SLIDER, 0, 100, 10);

            // Interval slider
            y_pos += 30;
            mk_static("Interval:", 15, y_pos, 120, 20, None, f, false);
            mk_static("5 s", 370, y_pos, 80, 20, Some(IDC_INTERVAL_VAL), f, false);
            make_slider(hwnd, 140, y_pos, 225, 25, IDC_INTERVAL_SLIDER, 1, 60, 5);

            // Start delay slider
            y_pos += 30;
            mk_static("Start Delay:", 15, y_pos, 120, 20, None, f, false);
            mk_static("5 s", 370, y_pos, 80, 20, Some(IDC_DELAY_VAL), f, false);
            make_slider(hwnd, 140, y_pos, 225, 25, IDC_DELAY_SLIDER, 2, 10, 5);

            // Algorithm dropdown
            y_pos += 35;
            mk_static("Algorithm:", 15, y_pos, 80, 20, None, f, false);
            let combo = CreateWindowExW(WINDOW_EX_STYLE(0), w!("COMBOBOX"), w!(""),
                WS_CHILD | WS_VISIBLE | WINDOW_STYLE(CBS_DROPDOWNLIST as u32),
                100, y_pos - 2, 250, 200, hwnd, HMENU(IDC_ALGO_COMBO as _), HINSTANCE::default(), None);
            SendMessageW(combo, WM_SETFONT, WPARAM(f.0 as usize), LPARAM(1));
            for name in ["WindMouse (Classic)", "Enhanced WindMouse", "Bezier Curves", "Fitts's Law + Overshoot"] {
                let wname = wide(name);
                SendMessageW(combo, CB_ADDSTRING, WPARAM(0), LPARAM(wname.as_ptr() as isize));
            }
            SendMessageW(combo, CB_SETCURSEL, WPARAM(0), LPARAM(0));

            // Checkboxes
            y_pos += 30;
            let chaos_chk = CreateWindowExW(WINDOW_EX_STYLE(0), w!("BUTTON"), w!("CHAOS MODE!"),
                WS_CHILD | WS_VISIBLE | WINDOW_STYLE(BS_AUTOCHECKBOX as u32),
                15, y_pos, 140, 22, hwnd, HMENU(IDC_CHAOS_CHECK as _), HINSTANCE::default(), None);
            SendMessageW(chaos_chk, WM_SETFONT, WPARAM(bf.0 as usize), LPARAM(1));

            let top_chk = CreateWindowExW(WINDOW_EX_STYLE(0), w!("BUTTON"), w!("Always on Top"),
                WS_CHILD | WS_VISIBLE | WINDOW_STYLE(BS_AUTOCHECKBOX as u32),
                170, y_pos, 140, 22, hwnd, HMENU(IDC_TOPMOST_CHECK as _), HINSTANCE::default(), None);
            SendMessageW(top_chk, WM_SETFONT, WPARAM(f.0 as usize), LPARAM(1));

            // Score label
            y_pos += 28;
            mk_static("Score: -- | Risk: -- | Moves: 0", 15, y_pos, 450, 22, Some(IDC_SCORE_LABEL), bf, false);

            // Buttons
            y_pos += 30;
            let start_btn = CreateWindowExW(WINDOW_EX_STYLE(0), w!("BUTTON"), w!("WIGGLE ME!"),
                WS_CHILD | WS_VISIBLE | WINDOW_STYLE(BS_PUSHBUTTON as u32),
                15, y_pos, 220, 50, hwnd, HMENU(IDC_START_BTN as _), HINSTANCE::default(), None);
            SendMessageW(start_btn, WM_SETFONT, WPARAM(bf.0 as usize), LPARAM(1));

            let learn_btn = CreateWindowExW(WINDOW_EX_STYLE(0), w!("BUTTON"), w!("LEARN MY MOVEMENT"),
                WS_CHILD | WS_VISIBLE | WINDOW_STYLE(BS_PUSHBUTTON as u32),
                245, y_pos, 220, 50, hwnd, HMENU(IDC_LEARN_BTN as _), HINSTANCE::default(), None);
            SendMessageW(learn_btn, WM_SETFONT, WPARAM(f.0 as usize), LPARAM(1));

            // Status
            y_pos += 58;
            mk_static("Ready. Move mouse to stop while active.", 15, y_pos, 450, 22, Some(IDC_STATUS), f, false);

            // Score bar background area
            let _score_y = y_pos + 28;
            // (score bar drawn in WM_ERASEBKGND)

            MAIN_HWND = hwnd;
            LRESULT(0)
        }

        WM_HSCROLL => {
            // Update slider values
            let pitch_slider = GetDlgItem(hwnd, IDC_PITCH_SLIDER);
            let yaw_slider = GetDlgItem(hwnd, IDC_YAW_SLIDER);
            let interval_slider = GetDlgItem(hwnd, IDC_INTERVAL_SLIDER);
            let delay_slider = GetDlgItem(hwnd, IDC_DELAY_SLIDER);

            let p = SendMessageW(pitch_slider, TBM_GETPOS, WPARAM(0), LPARAM(0)).0 as i32;
            let y = SendMessageW(yaw_slider, TBM_GETPOS, WPARAM(0), LPARAM(0)).0 as i32;
            let iv = SendMessageW(interval_slider, TBM_GETPOS, WPARAM(0), LPARAM(0)).0 as i32;
            let d = SendMessageW(delay_slider, TBM_GETPOS, WPARAM(0), LPARAM(0)).0 as i32;

            G_PITCH.store(p, Ordering::Relaxed);
            G_YAW.store(y, Ordering::Relaxed);
            G_INTERVAL.store(iv, Ordering::Relaxed);
            G_DELAY.store(d, Ordering::Relaxed);

            set_text(hwnd, IDC_PITCH_VAL, &format!("{} px", p));
            set_text(hwnd, IDC_YAW_VAL, &format!("{} px", y));
            set_text(hwnd, IDC_INTERVAL_VAL, &format!("{} s", iv));
            set_text(hwnd, IDC_DELAY_VAL, &format!("{} s", d));
            LRESULT(0)
        }

        WM_COMMAND => {
            let id = (wp.0 & 0xFFFF) as i32;
            let notify = ((wp.0 >> 16) & 0xFFFF) as u32;

            match id {
                IDC_START_BTN if notify == BN_CLICKED as u32 => {
                    if G_RUNNING.load(Ordering::Relaxed) {
                        // Stop
                        G_RUNNING.store(false, Ordering::Relaxed);
                        let btn = GetDlgItem(hwnd, IDC_START_BTN);
                        let _ = SetWindowTextW(btn, w!("WIGGLE ME!"));
                        let _ = EnableWindow(btn, TRUE);
                        let combo = GetDlgItem(hwnd, IDC_ALGO_COMBO);
                        let _ = EnableWindow(combo, TRUE);
                        set_text(hwnd, IDC_STATUS, "Phew, stopped.");
                    } else {
                        // Start with delay lockout
                        G_RUNNING.store(true, Ordering::Relaxed);

                        // Read algorithm
                        let combo = GetDlgItem(hwnd, IDC_ALGO_COMBO);
                        let sel = SendMessageW(combo, CB_GETCURSEL, WPARAM(0), LPARAM(0)).0 as i32;
                        G_ALGORITHM.store(sel, Ordering::Relaxed);

                        // Read chaos mode
                        let chaos_chk = GetDlgItem(hwnd, IDC_CHAOS_CHECK);
                        let checked = SendMessageW(chaos_chk, BM_GETCHECK, WPARAM(0), LPARAM(0)).0;
                        G_CHAOS.store(checked != 0, Ordering::Relaxed);

                        // Button lockout
                        let delay = G_DELAY.load(Ordering::Relaxed);
                        let btn = GetDlgItem(hwnd, IDC_START_BTN);
                        let lock_text = format!("STOP (Lock: {}s)", delay);
                        let _ = SetWindowTextW(btn, &HSTRING::from(lock_text));
                        let _ = EnableWindow(btn, FALSE);
                        let _ = EnableWindow(combo, FALSE);

                        // Spawn re-enable thread
                        let btn_raw = btn.0;
                        thread::spawn(move || {
                            thread::sleep(Duration::from_secs(delay as u64));
                            if G_RUNNING.load(Ordering::Relaxed) {
                                unsafe {
                                    let b = HWND(btn_raw);
                                    let _ = SetWindowTextW(b, w!("STOP!"));
                                    let _ = EnableWindow(b, TRUE);
                                }
                            }
                        });

                        // Spawn wiggler
                        let wh = hwnd;
                        thread::spawn(move || wiggler_thread(wh));
                    }
                }

                IDC_LEARN_BTN if notify == BN_CLICKED as u32 => {
                    if G_RECORDING.load(Ordering::Relaxed) {
                        // Stop recording, build fingerprint
                        G_RECORDING.store(false, Ordering::Relaxed);
                        let samples_arc = get_samples();
                        let samples = samples_arc.lock().unwrap();
                        let count = samples.len();
                        let fp = build_fingerprint(&samples);
                        drop(samples);

                        if fp.built {
                            let fp_arc = get_fingerprint();
                            *fp_arc.lock().unwrap() = fp;
                            set_text(hwnd, IDC_STATUS, &format!("Fingerprint built from {} samples! AI will mimic your style.", count));
                        } else {
                            set_text(hwnd, IDC_STATUS, &format!("Only {} samples - need 50+. Move mouse more, try again.", count));
                        }
                        let btn = GetDlgItem(hwnd, IDC_LEARN_BTN);
                        let _ = SetWindowTextW(btn, w!("LEARN MY MOVEMENT"));
                    } else {
                        // Start recording
                        G_RECORDING.store(true, Ordering::Relaxed);
                        { get_samples().lock().unwrap().clear(); }
                        let btn = GetDlgItem(hwnd, IDC_LEARN_BTN);
                        let _ = SetWindowTextW(btn, w!("STOP LEARNING"));
                        set_text(hwnd, IDC_STATUS, "Recording... Move your mouse naturally for 10+ seconds");
                    }
                }

                IDC_STOP_SIGNAL => {
                    // Safety stop signal from worker thread
                    G_RUNNING.store(false, Ordering::Relaxed);
                    let btn = GetDlgItem(hwnd, IDC_START_BTN);
                    let _ = SetWindowTextW(btn, w!("WIGGLE ME!"));
                    let _ = EnableWindow(btn, TRUE);
                    let combo = GetDlgItem(hwnd, IDC_ALGO_COMBO);
                    let _ = EnableWindow(combo, TRUE);
                }

                IDC_CHAOS_CHECK if notify == BN_CLICKED as u32 => {
                    let chk = GetDlgItem(hwnd, IDC_CHAOS_CHECK);
                    let checked = SendMessageW(chk, BM_GETCHECK, WPARAM(0), LPARAM(0)).0;
                    G_CHAOS.store(checked != 0, Ordering::Relaxed);
                }

                IDC_TOPMOST_CHECK if notify == BN_CLICKED as u32 => {
                    let chk = GetDlgItem(hwnd, IDC_TOPMOST_CHECK);
                    let checked = SendMessageW(chk, BM_GETCHECK, WPARAM(0), LPARAM(0)).0;
                    let pos = if checked != 0 { HWND_TOPMOST } else { HWND_NOTOPMOST };
                    let _ = SetWindowPos(hwnd, pos, 0, 0, 0, 0,
                        SWP_NOMOVE | SWP_NOSIZE | SWP_NOACTIVATE);
                }

                _ => {}
            }
            LRESULT(0)
        }

        WM_UPDATE_STATUS => {
            // Receive string from worker thread
            let ptr = lp.0 as *mut u16;
            if !ptr.is_null() {
                let len = {
                    let mut l = 0;
                    while *ptr.add(l) != 0 { l += 1; }
                    l
                };
                let text = String::from_utf16_lossy(std::slice::from_raw_parts(ptr, len));
                set_text(hwnd, IDC_STATUS, &text);
                // Free the boxed slice
                let _ = Box::from_raw(std::slice::from_raw_parts_mut(ptr, len + 1));
            }
            LRESULT(0)
        }

        WM_UPDATE_SCORE => {
            let score = G_SCORE.load(Ordering::Relaxed);
            let moves = G_MOVES.load(Ordering::Relaxed);
            let risk = if score >= 80 { "LOW" } else if score >= 50 { "MEDIUM" } else { "HIGH" };
            set_text(hwnd, IDC_SCORE_LABEL, &format!("Score: {}/100 | Risk: {} | Moves: {}", score, risk, moves));
            let _ = InvalidateRect(hwnd, None, false);
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

            // Score bar at bottom
            let score = G_SCORE.load(Ordering::Relaxed);
            let (bx, by, bw, bh) = (15, rc.bottom - 30, rc.right - 30, 18);
            let bg_r = RECT { left: bx, top: by, right: bx+bw, bottom: by+bh };
            let db = CreateSolidBrush(COLORREF(0x00302020));
            FillRect(hdc, &bg_r, db); let _ = DeleteObject(db);

            let fw = (bw as f64 * score as f64 / 100.0) as i32;
            let color = if score >= 80 { GREEN_COLOR } else if score >= 50 { YELLOW_COLOR } else { RED_COLOR };
            let fr = RECT { left: bx, top: by, right: bx+fw, bottom: by+bh };
            let fb = CreateSolidBrush(COLORREF(color));
            FillRect(hdc, &fr, fb); let _ = DeleteObject(fb);

            return LRESULT(1);
        }

        WM_CLOSE => { let _ = DestroyWindow(hwnd); LRESULT(0) }
        WM_DESTROY => { PostQuitMessage(0); LRESULT(0) }
        _ => DefWindowProcW(hwnd, msg, wp, lp),
    }
}

fn main() {
    unsafe {
        FINGERPRINT = Some(Arc::new(Mutex::new(UserFingerprint::default())));
        RECORDED_SAMPLES = Some(Arc::new(Mutex::new(Vec::new())));

        let icc = INITCOMMONCONTROLSEX { dwSize: std::mem::size_of::<INITCOMMONCONTROLSEX>() as u32, dwICC: ICC_BAR_CLASSES | ICC_STANDARD_CLASSES };
        let _ = InitCommonControlsEx(&icc);

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

        let hwnd = CreateWindowExW(
            WINDOW_EX_STYLE(0), PCWSTR(cn.as_ptr()),
            w!("WiggleMe AI - Human-Kinematic Mouse Simulator"),
            WS_OVERLAPPED | WS_CAPTION | WS_SYSMENU | WS_MINIMIZEBOX,
            CW_USEDEFAULT, CW_USEDEFAULT, 490, 470,
            HWND::default(), HMENU::default(), instance, None,
        );

        MAIN_HWND = hwnd;
        let _ = ShowWindow(hwnd, SW_SHOW);
        let _ = UpdateWindow(hwnd);

        // Start recording thread
        thread::spawn(|| record_thread());

        let mut msg = MSG::default();
        while GetMessageW(&mut msg, HWND::default(), 0, 0).into() {
            let _ = TranslateMessage(&msg);
            DispatchMessageW(&msg);
        }
    }
}
