// WiggleMe AI - AI Movement Engine Implementation
// Human-kinematic movement generation with behavioral fingerprinting

#include "../include/ai_movement.h"
#include <numeric>
#include <ctime>

// ============================================================================
// PerlinNoise1D Implementation
// ============================================================================
PerlinNoise1D::PerlinNoise1D() {
    std::mt19937 gen(std::random_device{}());
    int p[256];
    for (int i = 0; i < 256; i++) p[i] = i;
    for (int i = 255; i > 0; i--) {
        std::uniform_int_distribution<int> dist(0, i);
        std::swap(p[i], p[dist(gen)]);
    }
    for (int i = 0; i < 512; i++) perm[i] = p[i & 255];
}

double PerlinNoise1D::fade(double t) const {
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

double PerlinNoise1D::lerp(double a, double b, double t) const {
    return a + t * (b - a);
}

double PerlinNoise1D::grad(int hash, double x) const {
    return (hash & 1) ? x : -x;
}

double PerlinNoise1D::noise(double x) const {
    int xi = (int)floor(x) & 255;
    double xf = x - floor(x);
    double u = fade(xf);
    return lerp(grad(perm[xi], xf), grad(perm[xi + 1], xf - 1.0), u);
}

// ============================================================================
// AIMovementEngine Implementation
// ============================================================================
AIMovementEngine::AIMovementEngine()
    : m_rng(std::random_device{}())
    , m_speedDist(350.0, 120.0)
    , m_pauseDist(800.0, 400.0)
    , m_uniform(0.0, 1.0)
    , m_isRecording(false)
    , m_intensity(0.7)
    , m_detectionAvoidance(true)
    , m_smartPauses(true)
{
}

AIMovementEngine::~AIMovementEngine() {}

// ============================================================================
// Fingerprint Learning
// ============================================================================
void AIMovementEngine::startRecording() {
    m_isRecording = true;
    m_recordedSamples.clear();
    m_recordStart = std::chrono::steady_clock::now();
}

void AIMovementEngine::stopRecording() {
    m_isRecording = false;
}

void AIMovementEngine::addSample(const Point2D& pos, bool isClick) {
    if (!m_isRecording) return;

    auto now = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(now - m_recordStart).count();

    MovementSample sample;
    sample.pos = pos;
    sample.isClick = isClick;
    sample.timestamp = elapsed;

    // Calculate speed and angle from previous sample
    if (!m_recordedSamples.empty()) {
        auto& prev = m_recordedSamples.back();
        double dt = elapsed - prev.timestamp;
        if (dt > 0.001) {
            double dist = pos.distTo(prev.pos);
            sample.speed = dist / dt;
            sample.angle = atan2(pos.y - prev.pos.y, pos.x - prev.pos.x);
        } else {
            sample.speed = 0;
            sample.angle = 0;
        }
    } else {
        sample.speed = 0;
        sample.angle = 0;
    }

    m_recordedSamples.push_back(sample);
}

void AIMovementEngine::buildFingerprint() {
    if (m_recordedSamples.size() < 100) return; // Need enough data

    UserFingerprint fp;

    // --- Speed Analysis ---
    std::vector<double> speeds;
    for (auto& s : m_recordedSamples) {
        if (s.speed > 1.0 && s.speed < 5000.0) { // Filter outliers
            speeds.push_back(s.speed);
        }
    }

    if (!speeds.empty()) {
        double sum = std::accumulate(speeds.begin(), speeds.end(), 0.0);
        fp.avgSpeed = sum / speeds.size();

        double sqSum = 0;
        for (auto s : speeds) sqSum += (s - fp.avgSpeed) * (s - fp.avgSpeed);
        fp.speedStdDev = sqrt(sqSum / speeds.size());

        fp.maxSpeed = *std::max_element(speeds.begin(), speeds.end());
        fp.minSpeed = *std::min_element(speeds.begin(), speeds.end());
    }

    // --- Pause Analysis ---
    std::vector<double> pauses;
    for (size_t i = 1; i < m_recordedSamples.size(); i++) {
        double dt = m_recordedSamples[i].timestamp - m_recordedSamples[i-1].timestamp;
        double dist = m_recordedSamples[i].pos.distTo(m_recordedSamples[i-1].pos);
        if (dt > 0.2 && dist < 5.0) { // Pause = low movement over time
            pauses.push_back(dt * 1000.0); // ms
        }
    }

    if (!pauses.empty()) {
        double sum = std::accumulate(pauses.begin(), pauses.end(), 0.0);
        fp.avgPauseDuration = sum / pauses.size();
        double totalTime = m_recordedSamples.back().timestamp;
        fp.pauseFrequency = (pauses.size() / totalTime) * 60.0; // per minute

        double sqSum = 0;
        for (auto p : pauses) sqSum += (p - fp.avgPauseDuration) * (p - fp.avgPauseDuration);
        fp.pauseStdDev = sqrt(sqSum / pauses.size());
    }

    // --- Direction Bias ---
    std::vector<double> angles;
    for (auto& s : m_recordedSamples) {
        if (s.speed > 10.0) angles.push_back(s.angle);
    }
    if (!angles.empty()) {
        double sinSum = 0, cosSum = 0;
        for (auto a : angles) { sinSum += sin(a); cosSum += cos(a); }
        fp.directionBias = atan2(sinSum / angles.size(), cosSum / angles.size());
    }

    // --- Screen Coverage ---
    double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
    double sumX = 0, sumY = 0;
    for (auto& s : m_recordedSamples) {
        minX = (std::min)(minX, s.pos.x); maxX = (std::max)(maxX, s.pos.x);
        minY = (std::min)(minY, s.pos.y); maxY = (std::max)(maxY, s.pos.y);
        sumX += s.pos.x; sumY += s.pos.y;
    }
    fp.preferredX = sumX / m_recordedSamples.size();
    fp.preferredY = sumY / m_recordedSamples.size();

    int screenW = GetSystemMetrics(SM_CXSCREEN);
    int screenH = GetSystemMetrics(SM_CYSCREEN);
    if (screenW > 0 && screenH > 0) {
        fp.screenCoverage = ((maxX - minX) * (maxY - minY)) / (screenW * screenH);
    }

    fp.isValid = true;
    m_fingerprint = fp;

    // Update distributions to match user's fingerprint
    m_speedDist = std::normal_distribution<double>(fp.avgSpeed, fp.speedStdDev);
    m_pauseDist = std::normal_distribution<double>(fp.avgPauseDuration, fp.pauseStdDev);
}

// ============================================================================
// Movement Generation
// ============================================================================
std::vector<Point2D> AIMovementEngine::generateTrajectory(Point2D start, Point2D end) {
    double dist = start.distTo(end);

    // Choose algorithm based on distance and randomness
    double r = m_uniform(m_rng);

    std::vector<Point2D> trajectory;

    if (dist < 50) {
        // Short distance: direct with micro-jitter
        trajectory = windMouse(start.x, start.y, end.x, end.y);
    } else if (r < 0.5) {
        // WindMouse (organic, physics-based)
        trajectory = windMouse(start.x, start.y, end.x, end.y);
    } else {
        // Bezier curve (smooth arc)
        // Generate control points with natural curve
        double cx = (start.x + end.x) / 2.0;
        double cy = (start.y + end.y) / 2.0;
        double perpX = -(end.y - start.y);
        double perpY = (end.x - start.x);
        double len = sqrt(perpX * perpX + perpY * perpY);
        if (len > 0) { perpX /= len; perpY /= len; }

        double curvature = m_fingerprint.isValid ?
            m_fingerprint.curvePreference : 0.6;
        double offset = dist * curvature * (m_uniform(m_rng) * 0.5 + 0.1);
        if (m_uniform(m_rng) < 0.5) offset = -offset;

        Point2D ctrl1(cx + perpX * offset * 0.6, cy + perpY * offset * 0.6);
        Point2D ctrl2(cx + perpX * offset * 0.4, cy + perpY * offset * 0.4);

        int steps = (std::max)(10, (int)(dist / 3.0));
        trajectory = bezierCurve(start, ctrl1, ctrl2, end, steps);
    }

    // Add overshoot (10-30% of the time)
    if (m_uniform(m_rng) < (m_fingerprint.isValid ? m_fingerprint.avgOvershoot : 0.15)) {
        Point2D overshoot = generateOvershoot(end,
            Point2D(end.x - start.x, end.y - start.y));
        // Add overshoot then correction
        auto correction = windMouse(overshoot.x, overshoot.y, end.x, end.y);
        trajectory.insert(trajectory.end(), correction.begin(), correction.end());
    }

    // Add micro-jitter to every point
    for (auto& p : trajectory) {
        p = addMicroJitter(p);
    }

    return trajectory;
}

std::vector<Point2D> AIMovementEngine::generateIdleMovement(Point2D current) {
    // Small, natural drift movement (1-15 pixels)
    double driftDist = 1.0 + m_uniform(m_rng) * 14.0;
    double angle = m_uniform(m_rng) * 2.0 * 3.14159265;

    // Bias toward user's preferred screen area if fingerprinted
    if (m_fingerprint.isValid) {
        double toPrefX = m_fingerprint.preferredX - current.x;
        double toPrefY = m_fingerprint.preferredY - current.y;
        double prefAngle = atan2(toPrefY, toPrefX);
        // Blend random angle with preference (30% bias)
        angle = angle * 0.7 + prefAngle * 0.3;
    }

    Point2D target(
        current.x + cos(angle) * driftDist,
        current.y + sin(angle) * driftDist
    );

    // Clamp to screen
    int screenW = GetSystemMetrics(SM_CXSCREEN);
    int screenH = GetSystemMetrics(SM_CYSCREEN);
    target.x = (std::max)(0.0, (std::min)((double)screenW - 1, target.x));
    target.y = (std::max)(0.0, (std::min)((double)screenH - 1, target.y));

    return generateTrajectory(current, target);
}

std::vector<AIMovementEngine::TimedMovement> AIMovementEngine::generateActivitySession(
    Point2D startPos, double durationSeconds) {

    std::vector<TimedMovement> session;
    double elapsed = 0;
    Point2D currentPos = startPos;
    int screenW = GetSystemMetrics(SM_CXSCREEN);
    int screenH = GetSystemMetrics(SM_CYSCREEN);

    double intensity = getCurrentIntensity() * m_intensity;

    while (elapsed < durationSeconds) {
        TimedMovement tm;

        // Generate pause before movement
        tm.delayBeforeMs = generatePauseDuration();

        // Scale pause by inverse of intensity (low intensity = longer pauses)
        if (intensity > 0.01) {
            tm.delayBeforeMs /= intensity;
        } else {
            tm.delayBeforeMs = 5000; // Very long pause when low intensity
        }

        // Clamp pause to realistic range
        tm.delayBeforeMs = (std::max)(200.0, (std::min)(15000.0, tm.delayBeforeMs));

        elapsed += tm.delayBeforeMs / 1000.0;
        if (elapsed >= durationSeconds) break;

        // Decide movement type
        double r = m_uniform(m_rng);
        if (r < 0.6) {
            // Idle drift (most common during "active" but not really working)
            auto drift = generateIdleMovement(currentPos);
            if (!drift.empty()) {
                tm.target = drift.back();
            } else {
                tm.target = currentPos;
            }
            tm.includeClick = false;
        } else if (r < 0.85) {
            // Move to random screen area (simulates looking at something)
            double targetX = m_uniform(m_rng) * screenW * 0.8 + screenW * 0.1;
            double targetY = m_uniform(m_rng) * screenH * 0.8 + screenH * 0.1;

            // Bias toward preferred area if fingerprinted
            if (m_fingerprint.isValid) {
                targetX = targetX * 0.5 + m_fingerprint.preferredX * 0.5;
                targetY = targetY * 0.5 + m_fingerprint.preferredY * 0.5;
            }

            tm.target = Point2D(targetX, targetY);
            tm.includeClick = (m_uniform(m_rng) < 0.3); // 30% chance of click
        } else {
            // Move to common UI elements (taskbar, close button area, etc.)
            double areas[][2] = {
                {(double)screenW / 2, (double)screenH - 25},  // Taskbar center
                {(double)screenW - 50, 15},                     // Close button area
                {150, (double)screenH / 2},                     // Left panel
                {(double)screenW / 2, (double)screenH / 2},   // Center
            };
            int idx = (int)(m_uniform(m_rng) * 4) % 4;
            tm.target = Point2D(areas[idx][0], areas[idx][1]);
            tm.includeClick = (m_uniform(m_rng) < 0.15);
        }

        currentPos = tm.target;
        session.push_back(tm);

        // Estimate movement time
        elapsed += currentPos.distTo(tm.target) / generateSpeed() + 0.1;
    }

    return session;
}

// ============================================================================
// Authenticity Scoring
// ============================================================================
double AIMovementEngine::calculateAuthenticityScore(const std::vector<Point2D>& trajectory) {
    if (trajectory.size() < 3) return 0.0;

    double score = 0.0;
    int checks = 0;

    // 1. Speed variance check (25 points)
    // Humans have variable speed; bots have constant speed
    std::vector<double> speeds;
    for (size_t i = 1; i < trajectory.size(); i++) {
        double dist = trajectory[i].distTo(trajectory[i-1]);
        speeds.push_back(dist); // Approximate speed between points
    }
    if (speeds.size() > 2) {
        double avgSpd = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
        double variance = 0;
        for (auto s : speeds) variance += (s - avgSpd) * (s - avgSpd);
        variance /= speeds.size();
        double cv = (avgSpd > 0) ? sqrt(variance) / avgSpd : 0; // Coefficient of variation
        // Humans typically have CV of 0.3-0.8
        if (cv >= 0.2 && cv <= 1.0) score += 25.0;
        else if (cv >= 0.1) score += 15.0;
        else score += 5.0;
        checks++;
    }

    // 2. Curve smoothness check (25 points)
    // Humans make smooth curves; bots make straight lines or jagged paths
    double totalAngleChange = 0;
    int angleCount = 0;
    for (size_t i = 2; i < trajectory.size(); i++) {
        double dx1 = trajectory[i-1].x - trajectory[i-2].x;
        double dy1 = trajectory[i-1].y - trajectory[i-2].y;
        double dx2 = trajectory[i].x - trajectory[i-1].x;
        double dy2 = trajectory[i].y - trajectory[i-1].y;
        double a1 = atan2(dy1, dx1);
        double a2 = atan2(dy2, dx2);
        double diff = fabs(a2 - a1);
        if (diff > 3.14159) diff = 2 * 3.14159 - diff;
        totalAngleChange += diff;
        angleCount++;
    }
    if (angleCount > 0) {
        double avgAngle = totalAngleChange / angleCount;
        // Smooth curves: 0.01-0.15 avg angle change
        if (avgAngle >= 0.005 && avgAngle <= 0.3) score += 25.0;
        else if (avgAngle < 0.005) score += 10.0; // Too straight
        else score += 10.0; // Too jagged
        checks++;
    }

    // 3. Acceleration pattern check (25 points)
    // Humans accelerate at start, decelerate at end
    if (speeds.size() >= 6) {
        size_t third = speeds.size() / 3;
        double startAvg = 0, midAvg = 0, endAvg = 0;
        for (size_t i = 0; i < third; i++) startAvg += speeds[i];
        for (size_t i = third; i < 2 * third; i++) midAvg += speeds[i];
        for (size_t i = 2 * third; i < speeds.size(); i++) endAvg += speeds[i];
        startAvg /= third; midAvg /= third; endAvg /= (speeds.size() - 2 * third);

        // Natural: start slow, peak mid, slow at end
        if (midAvg > startAvg * 0.8 && midAvg > endAvg * 0.8) score += 25.0;
        else score += 12.0;
        checks++;
    }

    // 4. Micro-jitter check (25 points)
    // Humans have small, natural tremor/jitter
    int jitterCount = 0;
    for (size_t i = 1; i < trajectory.size(); i++) {
        double dist = trajectory[i].distTo(trajectory[i-1]);
        if (dist > 0.1 && dist < 3.0) jitterCount++;
    }
    double jitterRatio = (double)jitterCount / trajectory.size();
    if (jitterRatio >= 0.05 && jitterRatio <= 0.4) score += 25.0;
    else if (jitterRatio > 0) score += 12.0;
    checks++;

    return (checks > 0) ? (score / checks) * (100.0 / 25.0) : 0.0;
}

std::string AIMovementEngine::getAuthenticityReport(const std::vector<Point2D>& trajectory) {
    double score = calculateAuthenticityScore(trajectory);
    std::string report;
    report += "Authenticity Score: " + std::to_string((int)score) + "/100\n";

    if (score >= 85) report += "Rating: EXCELLENT - Virtually indistinguishable from human\n";
    else if (score >= 70) report += "Rating: GOOD - Passes most detection systems\n";
    else if (score >= 50) report += "Rating: MODERATE - May trigger advanced detection\n";
    else report += "Rating: LOW - Likely detectable by monitoring software\n";

    report += "Risk Level: ";
    if (score >= 80) report += "LOW";
    else if (score >= 60) report += "MEDIUM";
    else report += "HIGH";

    return report;
}

// ============================================================================
// Temporal Awareness
// ============================================================================
bool AIMovementEngine::shouldBeActive() const {
    time_t now = time(nullptr);
    struct tm lt;
    localtime_s(&lt, &now);

    // Check weekend
    if (!m_temporal.enableWeekends && (lt.tm_wday == 0 || lt.tm_wday == 6)) {
        return false;
    }

    // Check work hours
    int hour = lt.tm_hour;
    return (hour >= m_temporal.workStartHour && hour < m_temporal.workEndHour);
}

double AIMovementEngine::getCurrentIntensity() const {
    time_t now = time(nullptr);
    struct tm lt;
    localtime_s(&lt, &now);

    int hour = lt.tm_hour;
    if (hour < 0 || hour > 23) return 0.0;

    double baseIntensity = m_temporal.hourlyIntensity[hour];

    // Add minute-level variation (not exactly on the hour)
    double minuteVar = m_perlin.noise(lt.tm_min / 10.0) * 0.1;
    baseIntensity += minuteVar;

    // Apply fingerprint time-of-day multipliers
    if (m_fingerprint.isValid) {
        if (hour < 12) baseIntensity *= m_fingerprint.morningSpeed;
        else baseIntensity *= m_fingerprint.afternoonSpeed;
    }

    return (std::max)(0.0, (std::min)(1.0, baseIntensity));
}

void AIMovementEngine::setTemporalProfile(const TemporalProfile& profile) {
    m_temporal = profile;
}

void AIMovementEngine::setIntensity(double intensity) {
    m_intensity = (std::max)(0.0, (std::min)(1.0, intensity));
}

void AIMovementEngine::setDetectionAvoidance(bool enabled) {
    m_detectionAvoidance = enabled;
}

void AIMovementEngine::setSmartPauses(bool enabled) {
    m_smartPauses = enabled;
}

// ============================================================================
// Internal: WindMouse Algorithm
// ============================================================================
std::vector<Point2D> AIMovementEngine::windMouse(
    double sx, double sy, double ex, double ey) {

    std::vector<Point2D> points;
    points.push_back(Point2D(sx, sy));

    double dist = sqrt((ex - sx) * (ex - sx) + (ey - sy) * (ey - sy));
    if (dist < 1.0) { points.push_back(Point2D(ex, ey)); return points; }

    // Physics parameters (tuned for human-like feel)
    double gravity = 9.0 + m_uniform(m_rng) * 3.0;
    double windForce = 3.0 + m_uniform(m_rng) * 2.0;
    double maxStep = dist / 4.0 + m_uniform(m_rng) * 5.0;
    double targetArea = 8.0 + m_uniform(m_rng) * 4.0;

    double windX = 0, windY = 0;
    double veloX = 0, veloY = 0;
    double x = sx, y = sy;
    int maxIter = 1000;

    while (dist > 1.0 && maxIter-- > 0) {
        // Wind force (random organic deviation)
        double windMag = (std::min)(windForce, dist);
        if (dist >= targetArea) {
            windX = windX / 1.4 + (m_uniform(m_rng) * 2.0 - 1.0) * windMag / 2.0;
            windY = windY / 1.4 + (m_uniform(m_rng) * 2.0 - 1.0) * windMag / 2.0;
        } else {
            windX /= 1.4;
            windY /= 1.4;
        }

        // Gravity (pulls toward target)
        veloX += windX + gravity * (ex - x) / dist;
        veloY += windY + gravity * (ey - y) / dist;

        // Speed limit
        double veloMag = sqrt(veloX * veloX + veloY * veloY);
        if (veloMag > maxStep) {
            double scale = maxStep / veloMag;
            // Add Perlin noise to speed (organic variation)
            double noiseScale = 1.0 + m_perlin.noise(x * 0.01 + y * 0.01) * 0.2;
            scale *= noiseScale;
            veloX *= scale;
            veloY *= scale;
        }

        x += veloX;
        y += veloY;
        dist = sqrt((ex - x) * (ex - x) + (ey - y) * (ey - y));

        points.push_back(Point2D(x, y));
    }

    points.push_back(Point2D(ex, ey));
    return points;
}

// ============================================================================
// Internal: Bezier Curve
// ============================================================================
std::vector<Point2D> AIMovementEngine::bezierCurve(
    Point2D p0, Point2D p1, Point2D p2, Point2D p3, int steps) {

    std::vector<Point2D> points;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;

        // Apply easing (slow start, fast middle, slow end)
        // Cubic ease-in-out
        double easedT;
        if (t < 0.5) {
            easedT = 4.0 * t * t * t;
        } else {
            easedT = 1.0 - pow(-2.0 * t + 2.0, 3.0) / 2.0;
        }

        double u = 1.0 - easedT;
        double tt = easedT * easedT;
        double uu = u * u;
        double uuu = uu * u;
        double ttt = tt * easedT;

        Point2D p;
        p.x = uuu * p0.x + 3 * uu * easedT * p1.x + 3 * u * tt * p2.x + ttt * p3.x;
        p.y = uuu * p0.y + 3 * uu * easedT * p1.y + 3 * u * tt * p2.y + ttt * p3.y;
        points.push_back(p);
    }
    return points;
}

// ============================================================================
// Internal: Micro-Jitter (Human hand tremor simulation)
// ============================================================================
Point2D AIMovementEngine::addMicroJitter(Point2D p) {
    // Sub-pixel natural hand tremor (0.3-1.5px)
    double jitterMag = 0.3 + m_uniform(m_rng) * 1.2;
    double angle = m_perlin.noise(p.x * 0.05 + p.y * 0.05) * 2.0 * 3.14159;
    p.x += cos(angle) * jitterMag;
    p.y += sin(angle) * jitterMag;
    return p;
}

// ============================================================================
// Internal: Pause Duration Generation
// ============================================================================
double AIMovementEngine::generatePauseDuration() {
    double pause = m_pauseDist(m_rng);
    // Clamp to realistic range
    pause = (std::max)(100.0, (std::min)(8000.0, fabs(pause)));

    // Add occasional long pauses (thinking, reading)
    if (m_uniform(m_rng) < 0.1) {
        pause += 2000.0 + m_uniform(m_rng) * 5000.0; // 2-7 extra seconds
    }

    return pause;
}

// ============================================================================
// Internal: Speed Generation
// ============================================================================
double AIMovementEngine::generateSpeed() {
    double speed = m_speedDist(m_rng);
    return (std::max)(30.0, (std::min)(1200.0, fabs(speed)));
}

// ============================================================================
// Internal: Overshoot Generation
// ============================================================================
Point2D AIMovementEngine::generateOvershoot(Point2D target, Point2D direction) {
    double len = sqrt(direction.x * direction.x + direction.y * direction.y);
    if (len < 1.0) return target;

    double nx = direction.x / len;
    double ny = direction.y / len;

    // Overshoot by 5-25 pixels past target
    double overshootDist = 5.0 + m_uniform(m_rng) * 20.0;

    // Add slight perpendicular deviation
    double perpDist = (m_uniform(m_rng) - 0.5) * 10.0;

    return Point2D(
        target.x + nx * overshootDist - ny * perpDist,
        target.y + ny * overshootDist + nx * perpDist
    );
}
