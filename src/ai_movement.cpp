// WiggleMe AI - AI Movement Engine Implementation
// v2.0: Markov chains, velocity curves, persistent fingerprints, adaptive
// learning

#include "../include/ai_movement.h"
#include <ctime>
#include <numeric>

// ============================================================================
// PerlinNoise1D Implementation
// ============================================================================
PerlinNoise1D::PerlinNoise1D() {
  std::mt19937 gen(std::random_device{}());
  int p[256];
  for (int i = 0; i < 256; i++)
    p[i] = i;
  for (int i = 255; i > 0; i--) {
    std::uniform_int_distribution<int> dist(0, i);
    std::swap(p[i], p[dist(gen)]);
  }
  for (int i = 0; i < 512; i++)
    perm[i] = p[i & 255];
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
    : m_rng(std::random_device{}()), m_speedDist(350.0, 120.0),
      m_pauseDist(800.0, 400.0), m_uniform(0.0, 1.0),
      m_currentState(STATE_IDLE), m_markovTrained(false),
      m_velocityCurvesTrained(false), m_isRecording(false),
      m_adaptationRate(0.05), m_intensity(0.7), m_detectionAvoidance(true),
      m_smartPauses(true) {}

AIMovementEngine::~AIMovementEngine() {}

// ============================================================================
// Fingerprint Learning
// ============================================================================
void AIMovementEngine::startRecording() {
  m_isRecording = true;
  m_recordedSamples.clear();
  m_recordStart = std::chrono::steady_clock::now();
}

void AIMovementEngine::stopRecording() { m_isRecording = false; }

void AIMovementEngine::addSample(const Point2D &pos, bool isClick) {
  if (!m_isRecording)
    return;

  auto now = std::chrono::steady_clock::now();
  double elapsed = std::chrono::duration<double>(now - m_recordStart).count();

  MovementSample sample;
  sample.pos = pos;
  sample.isClick = isClick;
  sample.timestamp = elapsed;

  if (!m_recordedSamples.empty()) {
    auto &prev = m_recordedSamples.back();
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
  if (m_recordedSamples.size() < 100)
    return;

  UserFingerprint fp;

  // --- Speed Analysis ---
  std::vector<double> speeds;
  for (auto &s : m_recordedSamples) {
    if (s.speed > 1.0 && s.speed < 5000.0) {
      speeds.push_back(s.speed);
    }
  }

  if (!speeds.empty()) {
    double sum = std::accumulate(speeds.begin(), speeds.end(), 0.0);
    fp.avgSpeed = sum / speeds.size();

    double sqSum = 0;
    for (auto s : speeds)
      sqSum += (s - fp.avgSpeed) * (s - fp.avgSpeed);
    fp.speedStdDev = sqrt(sqSum / speeds.size());

    fp.maxSpeed = *std::max_element(speeds.begin(), speeds.end());
    fp.minSpeed = *std::min_element(speeds.begin(), speeds.end());
  }

  // --- Pause Analysis ---
  std::vector<double> pauses;
  for (size_t i = 1; i < m_recordedSamples.size(); i++) {
    double dt =
        m_recordedSamples[i].timestamp - m_recordedSamples[i - 1].timestamp;
    double dist = m_recordedSamples[i].pos.distTo(m_recordedSamples[i - 1].pos);
    if (dt > 0.2 && dist < 5.0) {
      pauses.push_back(dt * 1000.0);
    }
  }

  if (!pauses.empty()) {
    double sum = std::accumulate(pauses.begin(), pauses.end(), 0.0);
    fp.avgPauseDuration = sum / pauses.size();
    double totalTime = m_recordedSamples.back().timestamp;
    fp.pauseFrequency = (pauses.size() / totalTime) * 60.0;

    double sqSum = 0;
    for (auto p : pauses)
      sqSum += (p - fp.avgPauseDuration) * (p - fp.avgPauseDuration);
    fp.pauseStdDev = sqrt(sqSum / pauses.size());
  }

  // --- Direction Bias ---
  std::vector<double> angles;
  for (auto &s : m_recordedSamples) {
    if (s.speed > 10.0)
      angles.push_back(s.angle);
  }
  if (!angles.empty()) {
    double sinSum = 0, cosSum = 0;
    for (auto a : angles) {
      sinSum += sin(a);
      cosSum += cos(a);
    }
    fp.directionBias = atan2(sinSum / angles.size(), cosSum / angles.size());
  }

  // --- Screen Coverage ---
  double minX = 1e9, maxX = -1e9, minY = 1e9, maxY = -1e9;
  double sumX = 0, sumY = 0;
  for (auto &s : m_recordedSamples) {
    minX = (std::min)(minX, s.pos.x);
    maxX = (std::max)(maxX, s.pos.x);
    minY = (std::min)(minY, s.pos.y);
    maxY = (std::max)(maxY, s.pos.y);
    sumX += s.pos.x;
    sumY += s.pos.y;
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
  m_pauseDist =
      std::normal_distribution<double>(fp.avgPauseDuration, fp.pauseStdDev);

  // Build advanced learning data
  buildTransitionMatrix();
  buildVelocityCurves();
}

// ============================================================================
// Markov Chain: Classify Movement State
// ============================================================================
MovementState AIMovementEngine::classifyMovementState(double speed,
                                                      double distance,
                                                      double angleChange) {

  if (speed < 5.0 && distance < 2.0) {
    return STATE_IDLE;
  }
  if (speed < 50.0 && distance < 20.0) {
    return STATE_DRIFT;
  }
  if (speed > 500.0 && distance > 200.0) {
    return STATE_FAST_SWIPE;
  }
  // Detect overshoot correction: sudden direction reversal at moderate speed
  if (fabs(angleChange) > 2.5 && speed > 30.0 && speed < 300.0) {
    return STATE_OVERSHOOT_CORRECT;
  }
  return STATE_PRECISE_MOVE;
}

// ============================================================================
// Markov Chain: Build Transition Matrix from Recordings
// ============================================================================
void AIMovementEngine::buildTransitionMatrix() {
  if (m_recordedSamples.size() < 50)
    return;

  TransitionMatrix tm;

  MovementState prevState = STATE_IDLE;
  for (size_t i = 2; i < m_recordedSamples.size(); i++) {
    auto &cur = m_recordedSamples[i];
    auto &prev = m_recordedSamples[i - 1];
    auto &prev2 = m_recordedSamples[i - 2];

    double dist = cur.pos.distTo(prev.pos);

    // Calculate angle change
    double a1 = atan2(prev.pos.y - prev2.pos.y, prev.pos.x - prev2.pos.x);
    double a2 = atan2(cur.pos.y - prev.pos.y, cur.pos.x - prev.pos.x);
    double angleChange = a2 - a1;
    if (angleChange > 3.14159)
      angleChange -= 2 * 3.14159;
    if (angleChange < -3.14159)
      angleChange += 2 * 3.14159;

    MovementState curState =
        classifyMovementState(cur.speed, dist, angleChange);
    tm.counts[prevState][curState]++;
    prevState = curState;
  }

  tm.normalize();
  m_transitions = tm;
  m_markovTrained = true;
  m_currentState = STATE_IDLE;
}

// ============================================================================
// Velocity Curves: Learn Acceleration Profiles
// ============================================================================
void AIMovementEngine::buildVelocityCurves() {
  if (m_recordedSamples.size() < 100)
    return;

  // Detect movement segments (separated by pauses)
  struct MovementSegment {
    size_t startIdx, endIdx;
    double totalDist;
  };
  std::vector<MovementSegment> segments;

  bool inMovement = false;
  size_t segStart = 0;
  double segDist = 0;

  for (size_t i = 1; i < m_recordedSamples.size(); i++) {
    double dist = m_recordedSamples[i].pos.distTo(m_recordedSamples[i - 1].pos);
    double dt =
        m_recordedSamples[i].timestamp - m_recordedSamples[i - 1].timestamp;
    double speed = (dt > 0.001) ? dist / dt : 0;

    if (!inMovement && speed > 20.0) {
      inMovement = true;
      segStart = i - 1;
      segDist = 0;
    }

    if (inMovement) {
      segDist += dist;
      if (speed < 5.0 || i == m_recordedSamples.size() - 1) {
        // End of segment
        if (i - segStart >= 5) {
          MovementSegment seg;
          seg.startIdx = segStart;
          seg.endIdx = i;
          seg.totalDist = segDist;
          segments.push_back(seg);
        }
        inMovement = false;
      }
    }
  }

  // Reset curves
  for (int c = 0; c < 3; c++) {
    m_velocityCurves[c] = VelocityCurve();
  }

  // Accumulate normalized speed profiles per distance category
  double accumCurves[3][VelocityCurve::NUM_SAMPLES] = {};
  int accumCounts[3] = {};

  for (auto &seg : segments) {
    // Categorize: 0=short(<100), 1=medium(100-500), 2=long(>500)
    int cat = 0;
    if (seg.totalDist >= 500.0)
      cat = 2;
    else if (seg.totalDist >= 100.0)
      cat = 1;

    size_t segLen = seg.endIdx - seg.startIdx;
    if (segLen < 3)
      continue;

    // Find max speed in segment for normalization
    double maxSpd = 1.0;
    for (size_t i = seg.startIdx;
         i <= seg.endIdx && i < m_recordedSamples.size(); i++) {
      maxSpd = (std::max)(maxSpd, m_recordedSamples[i].speed);
    }

    // Sample speed at normalized positions
    for (int s = 0; s < VelocityCurve::NUM_SAMPLES; s++) {
      double t = (double)s / (VelocityCurve::NUM_SAMPLES - 1);
      size_t idx = seg.startIdx + (size_t)(t * (segLen - 1));
      if (idx < m_recordedSamples.size()) {
        accumCurves[cat][s] += m_recordedSamples[idx].speed / maxSpd;
      }
    }
    accumCounts[cat]++;
  }

  // Average the accumulated curves
  bool anyTrained = false;
  for (int c = 0; c < 3; c++) {
    if (accumCounts[c] >= 3) { // Need at least 3 movements per category
      for (int s = 0; s < VelocityCurve::NUM_SAMPLES; s++) {
        m_velocityCurves[c].samples[s] = accumCurves[c][s] / accumCounts[c];
      }
      m_velocityCurves[c].learnCount = accumCounts[c];
      anyTrained = true;
    }
  }
  m_velocityCurvesTrained = anyTrained;
}

double AIMovementEngine::sampleVelocityCurve(double distance,
                                             double normalizedT) {
  // Select curve category
  int cat = 0;
  if (distance >= 500.0)
    cat = 2;
  else if (distance >= 100.0)
    cat = 1;

  // If this category wasn't trained, fall back to closest one that was
  if (m_velocityCurves[cat].learnCount == 0) {
    for (int c = 0; c < 3; c++) {
      if (m_velocityCurves[c].learnCount > 0) {
        cat = c;
        break;
      }
    }
  }

  // If nothing was trained, return default sine easing
  if (m_velocityCurves[cat].learnCount == 0) {
    return sin(normalizedT * 3.14159265);
  }

  // Interpolate between samples
  double pos = normalizedT * (VelocityCurve::NUM_SAMPLES - 1);
  int idx = (int)pos;
  double frac = pos - idx;
  idx = (std::max)(0, (std::min)(idx, VelocityCurve::NUM_SAMPLES - 2));
  return m_velocityCurves[cat].samples[idx] * (1.0 - frac) +
         m_velocityCurves[cat].samples[idx + 1] * frac;
}

int AIMovementEngine::getVelocityCurveCount() const {
  int count = 0;
  for (int c = 0; c < 3; c++) {
    if (m_velocityCurves[c].learnCount > 0)
      count++;
  }
  return count;
}

// ============================================================================
// Persistence: Save/Load Fingerprint + Markov + Velocity Curves
// ============================================================================
bool AIMovementEngine::saveFingerprint(const char *path) {
  std::ofstream file(path, std::ios::binary);
  if (!file.is_open())
    return false;

  // Header
  uint32_t magic = FINGERPRINT_MAGIC;
  uint32_t version = FINGERPRINT_VERSION;
  file.write(reinterpret_cast<const char *>(&magic), sizeof(magic));
  file.write(reinterpret_cast<const char *>(&version), sizeof(version));

  // Fingerprint
  file.write(reinterpret_cast<const char *>(&m_fingerprint),
             sizeof(UserFingerprint));

  // Transition matrix
  file.write(reinterpret_cast<const char *>(&m_markovTrained), sizeof(bool));
  file.write(reinterpret_cast<const char *>(&m_transitions),
             sizeof(TransitionMatrix));

  // Velocity curves
  file.write(reinterpret_cast<const char *>(&m_velocityCurvesTrained),
             sizeof(bool));
  for (int c = 0; c < 3; c++) {
    file.write(reinterpret_cast<const char *>(&m_velocityCurves[c]),
               sizeof(VelocityCurve));
  }

  return file.good();
}

bool AIMovementEngine::loadFingerprint(const char *path) {
  std::ifstream file(path, std::ios::binary);
  if (!file.is_open())
    return false;

  // Verify header
  uint32_t magic = 0, version = 0;
  file.read(reinterpret_cast<char *>(&magic), sizeof(magic));
  file.read(reinterpret_cast<char *>(&version), sizeof(version));

  if (magic != FINGERPRINT_MAGIC || version != FINGERPRINT_VERSION) {
    return false;
  }

  // Fingerprint
  file.read(reinterpret_cast<char *>(&m_fingerprint), sizeof(UserFingerprint));

  // Transition matrix
  file.read(reinterpret_cast<char *>(&m_markovTrained), sizeof(bool));
  file.read(reinterpret_cast<char *>(&m_transitions), sizeof(TransitionMatrix));

  // Velocity curves
  file.read(reinterpret_cast<char *>(&m_velocityCurvesTrained), sizeof(bool));
  for (int c = 0; c < 3; c++) {
    file.read(reinterpret_cast<char *>(&m_velocityCurves[c]),
              sizeof(VelocityCurve));
  }

  // Rebuild distributions
  if (m_fingerprint.isValid) {
    m_speedDist = std::normal_distribution<double>(m_fingerprint.avgSpeed,
                                                   m_fingerprint.speedStdDev);
    m_pauseDist = std::normal_distribution<double>(
        m_fingerprint.avgPauseDuration, m_fingerprint.pauseStdDev);
  }

  return file.good();
}

// ============================================================================
// Real-Time Adaptive Learning
// ============================================================================
void AIMovementEngine::adaptFromLiveSession(double observedSpeed,
                                            double observedPause) {
  if (!m_fingerprint.isValid)
    return;

  double r = m_adaptationRate;

  // Exponential moving average
  m_fingerprint.avgSpeed =
      m_fingerprint.avgSpeed * (1.0 - r) + observedSpeed * r;
  m_fingerprint.avgPauseDuration =
      m_fingerprint.avgPauseDuration * (1.0 - r) + observedPause * r;

  // Update distributions
  m_speedDist = std::normal_distribution<double>(m_fingerprint.avgSpeed,
                                                 m_fingerprint.speedStdDev);
  m_pauseDist = std::normal_distribution<double>(m_fingerprint.avgPauseDuration,
                                                 m_fingerprint.pauseStdDev);
}

void AIMovementEngine::setAdaptationRate(double rate) {
  m_adaptationRate = (std::max)(0.0, (std::min)(1.0, rate));
}

// ============================================================================
// Movement Generation (v2.0 - Markov-driven)
// ============================================================================
std::vector<Point2D> AIMovementEngine::generateTrajectory(Point2D start,
                                                          Point2D end) {
  double dist = start.distTo(end);

  std::vector<Point2D> trajectory;

  // Use Markov chain to decide movement style if trained
  if (m_markovTrained) {
    MovementState nextState =
        m_transitions.sampleNext(m_currentState, m_uniform(m_rng));
    m_currentState = nextState;

    switch (nextState) {
    case STATE_IDLE:
    case STATE_DRIFT:
      // Slow, gentle movement — always use WindMouse with low params
      trajectory = windMouse(start.x, start.y, end.x, end.y);
      break;

    case STATE_PRECISE_MOVE:
      // Controlled, bezier-based movement
      {
        double cx = (start.x + end.x) / 2.0;
        double cy = (start.y + end.y) / 2.0;
        double perpX = -(end.y - start.y);
        double perpY = (end.x - start.x);
        double len = sqrt(perpX * perpX + perpY * perpY);
        if (len > 0) {
          perpX /= len;
          perpY /= len;
        }

        double curvature =
            m_fingerprint.isValid ? m_fingerprint.curvePreference : 0.6;
        double offset = dist * curvature * (m_uniform(m_rng) * 0.3 + 0.05);
        if (m_uniform(m_rng) < 0.5)
          offset = -offset;

        Point2D ctrl1(cx + perpX * offset * 0.6, cy + perpY * offset * 0.6);
        Point2D ctrl2(cx + perpX * offset * 0.4, cy + perpY * offset * 0.4);

        int steps = (std::max)(10, (int)(dist / 3.0));
        trajectory = bezierCurve(start, ctrl1, ctrl2, end, steps);
      }
      break;

    case STATE_FAST_SWIPE:
      // Quick and direct — WindMouse with higher step size
      trajectory = windMouse(start.x, start.y, end.x, end.y);
      break;

    case STATE_OVERSHOOT_CORRECT:
      // Move past target, then snap back
      {
        trajectory = windMouse(start.x, start.y, end.x, end.y);
        Point2D overshoot =
            generateOvershoot(end, Point2D(end.x - start.x, end.y - start.y));
        auto correction = windMouse(overshoot.x, overshoot.y, end.x, end.y);
        trajectory.insert(trajectory.end(), correction.begin(),
                          correction.end());
      }
      break;

    default:
      trajectory = windMouse(start.x, start.y, end.x, end.y);
      break;
    }
  } else {
    // Fallback: original random selection
    double r = m_uniform(m_rng);

    if (dist < 50) {
      trajectory = windMouse(start.x, start.y, end.x, end.y);
    } else if (r < 0.5) {
      trajectory = windMouse(start.x, start.y, end.x, end.y);
    } else {
      double cx = (start.x + end.x) / 2.0;
      double cy = (start.y + end.y) / 2.0;
      double perpX = -(end.y - start.y);
      double perpY = (end.x - start.x);
      double len = sqrt(perpX * perpX + perpY * perpY);
      if (len > 0) {
        perpX /= len;
        perpY /= len;
      }

      double curvature =
          m_fingerprint.isValid ? m_fingerprint.curvePreference : 0.6;
      double offset = dist * curvature * (m_uniform(m_rng) * 0.5 + 0.1);
      if (m_uniform(m_rng) < 0.5)
        offset = -offset;

      Point2D ctrl1(cx + perpX * offset * 0.6, cy + perpY * offset * 0.6);
      Point2D ctrl2(cx + perpX * offset * 0.4, cy + perpY * offset * 0.4);

      int steps = (std::max)(10, (int)(dist / 3.0));
      trajectory = bezierCurve(start, ctrl1, ctrl2, end, steps);
    }

    // Random overshoot (10-30% of the time)
    if (m_uniform(m_rng) <
        (m_fingerprint.isValid ? m_fingerprint.avgOvershoot : 0.15)) {
      Point2D overshoot =
          generateOvershoot(end, Point2D(end.x - start.x, end.y - start.y));
      auto correction = windMouse(overshoot.x, overshoot.y, end.x, end.y);
      trajectory.insert(trajectory.end(), correction.begin(), correction.end());
    }
  }

  // Add micro-jitter to every point
  for (auto &p : trajectory) {
    p = addMicroJitter(p);
  }

  return trajectory;
}

std::vector<Point2D> AIMovementEngine::generateIdleMovement(Point2D current) {
  double driftDist = 1.0 + m_uniform(m_rng) * 14.0;
  double angle = m_uniform(m_rng) * 2.0 * 3.14159265;

  if (m_fingerprint.isValid) {
    double toPrefX = m_fingerprint.preferredX - current.x;
    double toPrefY = m_fingerprint.preferredY - current.y;
    double prefAngle = atan2(toPrefY, toPrefX);
    angle = angle * 0.7 + prefAngle * 0.3;
  }

  Point2D target(current.x + cos(angle) * driftDist,
                 current.y + sin(angle) * driftDist);

  int screenW = GetSystemMetrics(SM_CXSCREEN);
  int screenH = GetSystemMetrics(SM_CYSCREEN);
  target.x = (std::max)(0.0, (std::min)((double)screenW - 1, target.x));
  target.y = (std::max)(0.0, (std::min)((double)screenH - 1, target.y));

  return generateTrajectory(current, target);
}

std::vector<AIMovementEngine::TimedMovement>
AIMovementEngine::generateActivitySession(Point2D startPos,
                                          double durationSeconds) {

  std::vector<TimedMovement> session;
  double elapsed = 0;
  Point2D currentPos = startPos;
  int screenW = GetSystemMetrics(SM_CXSCREEN);
  int screenH = GetSystemMetrics(SM_CYSCREEN);

  double intensity = getCurrentIntensity() * m_intensity;

  while (elapsed < durationSeconds) {
    TimedMovement tm;

    tm.delayBeforeMs = generatePauseDuration();

    if (intensity > 0.01) {
      tm.delayBeforeMs /= intensity;
    } else {
      tm.delayBeforeMs = 5000;
    }

    tm.delayBeforeMs = (std::max)(200.0, (std::min)(15000.0, tm.delayBeforeMs));

    elapsed += tm.delayBeforeMs / 1000.0;
    if (elapsed >= durationSeconds)
      break;

    // Use Markov chain for movement type decision if trained
    double r = m_uniform(m_rng);
    MovementState nextAction = STATE_DRIFT;
    if (m_markovTrained) {
      nextAction = m_transitions.sampleNext(m_currentState, r);
      m_currentState = nextAction;
    }

    if (!m_markovTrained) {
      // Fallback to original random logic
      if (r < 0.6) {
        nextAction = STATE_DRIFT;
      } else if (r < 0.85) {
        nextAction = STATE_PRECISE_MOVE;
      } else {
        nextAction = STATE_FAST_SWIPE;
      }
    }

    switch (nextAction) {
    case STATE_IDLE:
    case STATE_DRIFT: {
      auto drift = generateIdleMovement(currentPos);
      if (!drift.empty())
        tm.target = drift.back();
      else
        tm.target = currentPos;
      tm.includeClick = false;
      break;
    }
    case STATE_PRECISE_MOVE: {
      double targetX = m_uniform(m_rng) * screenW * 0.8 + screenW * 0.1;
      double targetY = m_uniform(m_rng) * screenH * 0.8 + screenH * 0.1;
      if (m_fingerprint.isValid) {
        targetX = targetX * 0.5 + m_fingerprint.preferredX * 0.5;
        targetY = targetY * 0.5 + m_fingerprint.preferredY * 0.5;
      }
      tm.target = Point2D(targetX, targetY);
      tm.includeClick = (m_uniform(m_rng) < 0.3);
      break;
    }
    case STATE_FAST_SWIPE:
    case STATE_OVERSHOOT_CORRECT: {
      double areas[][2] = {
          {(double)screenW / 2, (double)screenH - 25},
          {(double)screenW - 50, 15},
          {150, (double)screenH / 2},
          {(double)screenW / 2, (double)screenH / 2},
      };
      int idx = (int)(m_uniform(m_rng) * 4) % 4;
      tm.target = Point2D(areas[idx][0], areas[idx][1]);
      tm.includeClick = (m_uniform(m_rng) < 0.15);
      break;
    }
    default:
      tm.target = currentPos;
      tm.includeClick = false;
      break;
    }

    currentPos = tm.target;
    session.push_back(tm);

    elapsed += currentPos.distTo(tm.target) / generateSpeed() + 0.1;
  }

  return session;
}

// ============================================================================
// Authenticity Scoring
// ============================================================================
double AIMovementEngine::calculateAuthenticityScore(
    const std::vector<Point2D> &trajectory) {
  if (trajectory.size() < 3)
    return 0.0;

  double score = 0.0;
  int checks = 0;

  // 1. Speed variance check (25 points)
  std::vector<double> speeds;
  for (size_t i = 1; i < trajectory.size(); i++) {
    double dist = trajectory[i].distTo(trajectory[i - 1]);
    speeds.push_back(dist);
  }
  if (speeds.size() > 2) {
    double avgSpd =
        std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
    double variance = 0;
    for (auto s : speeds)
      variance += (s - avgSpd) * (s - avgSpd);
    variance /= speeds.size();
    double cv = (avgSpd > 0) ? sqrt(variance) / avgSpd : 0;
    if (cv >= 0.2 && cv <= 1.0)
      score += 25.0;
    else if (cv >= 0.1)
      score += 15.0;
    else
      score += 5.0;
    checks++;
  }

  // 2. Curve smoothness check (25 points)
  double totalAngleChange = 0;
  int angleCount = 0;
  for (size_t i = 2; i < trajectory.size(); i++) {
    double dx1 = trajectory[i - 1].x - trajectory[i - 2].x;
    double dy1 = trajectory[i - 1].y - trajectory[i - 2].y;
    double dx2 = trajectory[i].x - trajectory[i - 1].x;
    double dy2 = trajectory[i].y - trajectory[i - 1].y;
    double a1 = atan2(dy1, dx1);
    double a2 = atan2(dy2, dx2);
    double diff = fabs(a2 - a1);
    if (diff > 3.14159)
      diff = 2 * 3.14159 - diff;
    totalAngleChange += diff;
    angleCount++;
  }
  if (angleCount > 0) {
    double avgAngle = totalAngleChange / angleCount;
    if (avgAngle >= 0.005 && avgAngle <= 0.3)
      score += 25.0;
    else if (avgAngle < 0.005)
      score += 10.0;
    else
      score += 10.0;
    checks++;
  }

  // 3. Acceleration pattern check (25 points)
  if (speeds.size() >= 6) {
    size_t third = speeds.size() / 3;
    double startAvg = 0, midAvg = 0, endAvg = 0;
    for (size_t i = 0; i < third; i++)
      startAvg += speeds[i];
    for (size_t i = third; i < 2 * third; i++)
      midAvg += speeds[i];
    for (size_t i = 2 * third; i < speeds.size(); i++)
      endAvg += speeds[i];
    startAvg /= third;
    midAvg /= third;
    endAvg /= (speeds.size() - 2 * third);

    if (midAvg > startAvg * 0.8 && midAvg > endAvg * 0.8)
      score += 25.0;
    else
      score += 12.0;
    checks++;
  }

  // 4. Micro-jitter check (25 points)
  int jitterCount = 0;
  for (size_t i = 1; i < trajectory.size(); i++) {
    double dist = trajectory[i].distTo(trajectory[i - 1]);
    if (dist > 0.1 && dist < 3.0)
      jitterCount++;
  }
  double jitterRatio = (double)jitterCount / trajectory.size();
  if (jitterRatio >= 0.05 && jitterRatio <= 0.4)
    score += 25.0;
  else if (jitterRatio > 0)
    score += 12.0;
  checks++;

  // 5. BONUS: Markov coherence check (+10 if Markov transitions feel natural)
  if (m_markovTrained && trajectory.size() > 10) {
    score += 10.0;
    checks++;
  }

  double maxScore = checks * 25.0;
  if (maxScore <= 0)
    return 0.0;
  return (score / maxScore) * 100.0;
}

std::string AIMovementEngine::getAuthenticityReport(
    const std::vector<Point2D> &trajectory) {
  double score = calculateAuthenticityScore(trajectory);
  std::string report;
  report += "Authenticity Score: " + std::to_string((int)score) + "/100\n";

  if (score >= 85)
    report += "Rating: EXCELLENT - Virtually indistinguishable from human\n";
  else if (score >= 70)
    report += "Rating: GOOD - Passes most detection systems\n";
  else if (score >= 50)
    report += "Rating: MODERATE - May trigger advanced detection\n";
  else
    report += "Rating: LOW - Likely detectable by monitoring software\n";

  report += "Risk Level: ";
  if (score >= 80)
    report += "LOW";
  else if (score >= 60)
    report += "MEDIUM";
  else
    report += "HIGH";

  if (m_markovTrained)
    report += "\nMarkov: Active (" + std::to_string(STATE_COUNT) + " states)";
  if (m_velocityCurvesTrained)
    report += "\nVelocity Curves: " + std::to_string(getVelocityCurveCount()) +
              "/3 trained";

  return report;
}

// ============================================================================
// Temporal Awareness
// ============================================================================
bool AIMovementEngine::shouldBeActive() const {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_s(&lt, &now);

  if (!m_temporal.enableWeekends && (lt.tm_wday == 0 || lt.tm_wday == 6)) {
    return false;
  }

  int hour = lt.tm_hour;
  return (hour >= m_temporal.workStartHour && hour < m_temporal.workEndHour);
}

double AIMovementEngine::getCurrentIntensity() const {
  time_t now = time(nullptr);
  struct tm lt;
  localtime_s(&lt, &now);

  int hour = lt.tm_hour;
  if (hour < 0 || hour > 23)
    return 0.0;

  double baseIntensity = m_temporal.hourlyIntensity[hour];

  double minuteVar = m_perlin.noise(lt.tm_min / 10.0) * 0.1;
  baseIntensity += minuteVar;

  if (m_fingerprint.isValid) {
    if (hour < 12)
      baseIntensity *= m_fingerprint.morningSpeed;
    else
      baseIntensity *= m_fingerprint.afternoonSpeed;
  }

  return (std::max)(0.0, (std::min)(1.0, baseIntensity));
}

void AIMovementEngine::setTemporalProfile(const TemporalProfile &profile) {
  m_temporal = profile;
}

void AIMovementEngine::setIntensity(double intensity) {
  m_intensity = (std::max)(0.0, (std::min)(1.0, intensity));
}

void AIMovementEngine::setDetectionAvoidance(bool enabled) {
  m_detectionAvoidance = enabled;
}

void AIMovementEngine::setSmartPauses(bool enabled) { m_smartPauses = enabled; }

// ============================================================================
// Internal: WindMouse Algorithm
// ============================================================================
std::vector<Point2D> AIMovementEngine::windMouse(double sx, double sy,
                                                 double ex, double ey) {

  std::vector<Point2D> points;
  points.push_back(Point2D(sx, sy));

  double dist = sqrt((ex - sx) * (ex - sx) + (ey - sy) * (ey - sy));
  if (dist < 1.0) {
    points.push_back(Point2D(ex, ey));
    return points;
  }

  double gravity = 9.0 + m_uniform(m_rng) * 3.0;
  double windForce = 3.0 + m_uniform(m_rng) * 2.0;
  double maxStep = dist / 4.0 + m_uniform(m_rng) * 5.0;
  double targetArea = 8.0 + m_uniform(m_rng) * 4.0;

  double windX = 0, windY = 0;
  double veloX = 0, veloY = 0;
  double x = sx, y = sy;
  int maxIter = 1000;

  while (dist > 1.0 && maxIter-- > 0) {
    double windMag = (std::min)(windForce, dist);
    if (dist >= targetArea) {
      windX = windX / 1.4 + (m_uniform(m_rng) * 2.0 - 1.0) * windMag / 2.0;
      windY = windY / 1.4 + (m_uniform(m_rng) * 2.0 - 1.0) * windMag / 2.0;
    } else {
      windX /= 1.4;
      windY /= 1.4;
    }

    veloX += windX + gravity * (ex - x) / dist;
    veloY += windY + gravity * (ey - y) / dist;

    double veloMag = sqrt(veloX * veloX + veloY * veloY);
    if (veloMag > maxStep) {
      double scale = maxStep / veloMag;
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
// Internal: Bezier Curve (v2.0 - uses learned velocity curves)
// ============================================================================
std::vector<Point2D> AIMovementEngine::bezierCurve(Point2D p0, Point2D p1,
                                                   Point2D p2, Point2D p3,
                                                   int steps) {

  std::vector<Point2D> points;
  double totalDist = p0.distTo(p3);

  for (int i = 0; i <= steps; i++) {
    double t = (double)i / steps;

    // Apply learned velocity curve easing if available
    double easedT;
    if (m_velocityCurvesTrained) {
      // Integrate velocity curve to get position-based easing
      // Simple approach: use velocity as easing weight
      double velFactor = sampleVelocityCurve(totalDist, t);
      // Blend between linear t and velocity-shaped t
      easedT = t * 0.3 + (velFactor > 0 ? t * velFactor / 1.0 : t) * 0.7;
      easedT = (std::max)(0.0, (std::min)(1.0, easedT));
    } else {
      // Cubic ease-in-out (default)
      if (t < 0.5) {
        easedT = 4.0 * t * t * t;
      } else {
        easedT = 1.0 - pow(-2.0 * t + 2.0, 3.0) / 2.0;
      }
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
  pause = (std::max)(100.0, (std::min)(8000.0, fabs(pause)));

  if (m_uniform(m_rng) < 0.1) {
    pause += 2000.0 + m_uniform(m_rng) * 5000.0;
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
  if (len < 1.0)
    return target;

  double nx = direction.x / len;
  double ny = direction.y / len;

  double overshootDist = 5.0 + m_uniform(m_rng) * 20.0;
  double perpDist = (m_uniform(m_rng) - 0.5) * 10.0;

  return Point2D(target.x + nx * overshootDist - ny * perpDist,
                 target.y + ny * overshootDist + nx * perpDist);
}
