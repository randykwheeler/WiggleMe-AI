// WiggleMe AI - AI Movement Engine Header
// Human-kinematic movement generation with behavioral fingerprinting
// v2.0: Markov chains, velocity curves, persistent fingerprints, adaptive learning
#pragma once

#include <windows.h>
#include <cmath>
#include <random>
#include <vector>
#include <chrono>
#include <algorithm>
#include <atomic>
#include <string>
#include <fstream>
#include <cstdint>

// ============================================================================
// Point2D - Basic 2D point with velocity
// ============================================================================
struct Point2D {
    double x, y;
    Point2D() : x(0), y(0) {}
    Point2D(double x, double y) : x(x), y(y) {}
    double distTo(const Point2D& o) const {
        return sqrt((x - o.x) * (x - o.x) + (y - o.y) * (y - o.y));
    }
};

// ============================================================================
// MovementSample - A single recorded movement data point
// ============================================================================
struct MovementSample {
    Point2D pos;
    double speed;       // px/s at this point
    double angle;       // direction of movement (radians)
    double timestamp;   // seconds since recording start
    bool isClick;       // was there a click at this point?
};

// ============================================================================
// MovementState - Classified movement type for Markov chains
// ============================================================================
enum MovementState {
    STATE_IDLE = 0,             // No significant movement
    STATE_DRIFT,                // Small, slow drifting
    STATE_PRECISE_MOVE,         // Careful, targeted movement
    STATE_FAST_SWIPE,           // Quick cross-screen move
    STATE_OVERSHOOT_CORRECT,    // Overshoot then snap back
    STATE_COUNT                 // Must be last
};

// ============================================================================
// VelocityCurve - Learned acceleration/deceleration profile
// ============================================================================
struct VelocityCurve {
    static const int NUM_SAMPLES = 32;  // Normalized curve resolution
    double samples[NUM_SAMPLES];        // Speed multiplier at each % of movement
    int learnCount;                     // How many movements contributed

    VelocityCurve() : learnCount(0) {
        // Default: bell curve (slow start, fast middle, slow end)
        for (int i = 0; i < NUM_SAMPLES; i++) {
            double t = (double)i / (NUM_SAMPLES - 1);
            samples[i] = sin(t * 3.14159265);
        }
    }
};

// ============================================================================
// TransitionMatrix - Markov chain state transitions
// ============================================================================
struct TransitionMatrix {
    // counts[from][to] = how many times we transitioned from state A to B
    int counts[STATE_COUNT][STATE_COUNT];
    // probabilities[from][to] = normalized probability
    double probabilities[STATE_COUNT][STATE_COUNT];

    TransitionMatrix() {
        memset(counts, 0, sizeof(counts));
        // Default uniform probabilities
        for (int i = 0; i < STATE_COUNT; i++)
            for (int j = 0; j < STATE_COUNT; j++)
                probabilities[i][j] = 1.0 / STATE_COUNT;
    }

    void normalize() {
        for (int i = 0; i < STATE_COUNT; i++) {
            int rowTotal = 0;
            for (int j = 0; j < STATE_COUNT; j++) rowTotal += counts[i][j];
            if (rowTotal > 0) {
                for (int j = 0; j < STATE_COUNT; j++)
                    probabilities[i][j] = (double)counts[i][j] / rowTotal;
            }
        }
    }

    MovementState sampleNext(MovementState current, double randomVal) const {
        double cum = 0;
        for (int j = 0; j < STATE_COUNT; j++) {
            cum += probabilities[current][j];
            if (randomVal <= cum) return (MovementState)j;
        }
        return STATE_DRIFT; // fallback
    }
};

// ============================================================================
// UserFingerprint - Learned behavioral signature of a specific user
// ============================================================================
struct UserFingerprint {
    // Speed profile
    double avgSpeed;            // px/s
    double speedStdDev;         // variance in speed
    double maxSpeed;
    double minSpeed;

    // Pause behavior
    double avgPauseDuration;    // ms
    double pauseFrequency;      // pauses per minute
    double pauseStdDev;

    // Movement style
    double avgOvershoot;        // % past target
    double curvePreference;     // 0 = straight, 1 = very curved
    double directionBias;       // preferred direction (radians)
    double handedness;          // -1 = left, 1 = right (affects curve direction)

    // Screen area preferences
    double preferredX;          // avg X position
    double preferredY;          // avg Y position
    double screenCoverage;      // % of screen used

    // Timing
    double morningSpeed;        // multiplier for morning
    double afternoonSpeed;      // multiplier for afternoon

    bool isValid;               // has been calibrated

    UserFingerprint() : avgSpeed(350), speedStdDev(120), maxSpeed(800),
        minSpeed(50), avgPauseDuration(800), pauseFrequency(4.0),
        pauseStdDev(400), avgOvershoot(0.12), curvePreference(0.6),
        directionBias(0.3), handedness(1.0), preferredX(960),
        preferredY(540), screenCoverage(0.7), morningSpeed(1.1),
        afternoonSpeed(0.85), isValid(false) {}
};

// ============================================================================
// TemporalProfile - Time-of-day activity patterns
// ============================================================================
struct TemporalProfile {
    int workStartHour;      // e.g. 8
    int workEndHour;        // e.g. 17
    int lunchStartHour;     // e.g. 12
    int lunchEndHour;       // e.g. 13
    bool enableWeekends;    // false = no activity on weekends

    // Activity intensity multipliers by hour (0-23)
    double hourlyIntensity[24];

    TemporalProfile() : workStartHour(8), workEndHour(17),
        lunchStartHour(12), lunchEndHour(13), enableWeekends(false) {
        // Default: realistic work pattern
        for (int i = 0; i < 24; i++) hourlyIntensity[i] = 0.0;
        hourlyIntensity[8]  = 0.6;   // Arriving, settling in
        hourlyIntensity[9]  = 0.9;   // Morning peak
        hourlyIntensity[10] = 1.0;   // Peak productivity
        hourlyIntensity[11] = 0.95;  // Still strong
        hourlyIntensity[12] = 0.3;   // Lunch dip
        hourlyIntensity[13] = 0.7;   // Post-lunch recovery
        hourlyIntensity[14] = 0.85;  // Afternoon work
        hourlyIntensity[15] = 0.8;   // Slight decline
        hourlyIntensity[16] = 0.7;   // Winding down
        hourlyIntensity[17] = 0.4;   // Wrapping up
    }
};

// ============================================================================
// Perlin Noise (1D) - For organic jitter generation
// ============================================================================
class PerlinNoise1D {
public:
    PerlinNoise1D();
    double noise(double x) const;

private:
    int perm[512];
    double fade(double t) const;
    double lerp(double a, double b, double t) const;
    double grad(int hash, double x) const;
};

// ============================================================================
// AIMovementEngine - Core AI movement generation (v2.0)
// ============================================================================
class AIMovementEngine {
public:
    AIMovementEngine();
    ~AIMovementEngine();

    // --- Fingerprint Learning ---
    void startRecording();
    void stopRecording();
    void addSample(const Point2D& pos, bool isClick);
    void buildFingerprint();
    bool hasFingerprint() const { return m_fingerprint.isValid; }
    const UserFingerprint& getFingerprint() const { return m_fingerprint; }

    // --- Markov Chain ---
    const TransitionMatrix& getTransitions() const { return m_transitions; }
    bool hasMarkovData() const { return m_markovTrained; }
    MovementState getCurrentState() const { return m_currentState; }

    // --- Velocity Curves ---
    bool hasVelocityCurves() const { return m_velocityCurvesTrained; }
    int getVelocityCurveCount() const;

    // --- Persistence ---
    bool saveFingerprint(const char* path);
    bool loadFingerprint(const char* path);

    // --- Real-Time Adaptive Learning ---
    void adaptFromLiveSession(double observedSpeed, double observedPause);
    void setAdaptationRate(double rate);  // 0.0 = no adaptation, 1.0 = instant

    // --- Movement Generation ---
    std::vector<Point2D> generateTrajectory(Point2D start, Point2D end);
    std::vector<Point2D> generateIdleMovement(Point2D current);

    struct TimedMovement {
        Point2D target;
        double delayBeforeMs;
        bool includeClick;
    };
    std::vector<TimedMovement> generateActivitySession(
        Point2D startPos, double durationSeconds);

    // --- Authenticity Scoring ---
    double calculateAuthenticityScore(const std::vector<Point2D>& trajectory);
    std::string getAuthenticityReport(const std::vector<Point2D>& trajectory);

    // --- Temporal Awareness ---
    bool shouldBeActive() const;
    double getCurrentIntensity() const;
    void setTemporalProfile(const TemporalProfile& profile);

    // --- Configuration ---
    void setIntensity(double intensity);
    void setDetectionAvoidance(bool enabled);
    void setSmartPauses(bool enabled);

private:
    // --- Internal Generation Methods ---
    std::vector<Point2D> windMouse(double sx, double sy, double ex, double ey);
    std::vector<Point2D> bezierCurve(Point2D p0, Point2D p1, Point2D p2, Point2D p3, int steps);
    Point2D addMicroJitter(Point2D p);
    double generatePauseDuration();
    double generateSpeed();
    Point2D generateOvershoot(Point2D target, Point2D direction);

    // --- Markov / Learning Internals ---
    MovementState classifyMovementState(double speed, double distance, double angleChange);
    void buildTransitionMatrix();
    void buildVelocityCurves();
    double sampleVelocityCurve(double distance, double normalizedT);

    // --- State ---
    UserFingerprint m_fingerprint;
    TemporalProfile m_temporal;
    TransitionMatrix m_transitions;
    PerlinNoise1D m_perlin;
    std::mt19937 m_rng;
    std::normal_distribution<double> m_speedDist;
    std::normal_distribution<double> m_pauseDist;
    std::uniform_real_distribution<double> m_uniform;

    // Markov state
    MovementState m_currentState;
    bool m_markovTrained;

    // Velocity curves (3 distance categories: short <100px, medium 100-500px, long >500px)
    VelocityCurve m_velocityCurves[3];
    bool m_velocityCurvesTrained;

    // Recording state
    bool m_isRecording;
    std::vector<MovementSample> m_recordedSamples;
    std::chrono::steady_clock::time_point m_recordStart;

    // Adaptation
    double m_adaptationRate;

    // Config
    double m_intensity;
    bool m_detectionAvoidance;
    bool m_smartPauses;

    // File format magic
    static const uint32_t FINGERPRINT_MAGIC = 0x574D4149; // "WMAI"
    static const uint32_t FINGERPRINT_VERSION = 2;
};
