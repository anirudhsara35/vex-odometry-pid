#pragma once
// drive_curves.hpp — Joystick-to-velocity mappings for driver control
//
// Evolution across two seasons (3303D -> 3303C):
//   1. trueSpeed[] lookup table — 128 hand-built entries so the old motors
//      never saw a stall-inducing jump.
//   2. Exponential curve — one equation instead of a table:
//        y = 1.2 * (1.043)^x - 1.2 + 0.2x
//      fine control near zero, ramps toward full speed.
//   3. Final S-curve — gentle near zero (precision), steeper through the
//      middle (responsiveness), flattens near full power. Found through
//      Desmos iteration; implemented here as a normalized logistic.
//
// All functions map joystick input in [-127, 127] to output in [-127, 127],
// odd-symmetric (f(-x) = -f(x)).

#include <array>
#include <cmath>
#include <cstdlib>

namespace vexlib {

inline constexpr double kJoyMax = 127.0;

// The 3303D acceleration lookup table, verbatim from the notebook.
inline constexpr std::array<unsigned int, 128> kTrueSpeed = {
    0,  0,  0,  0,  0,  0,  0,  0,  0,  0,
    0,  21, 21, 21, 22, 22, 22, 23, 24, 24,
    25, 25, 25, 25, 26, 27, 27, 28, 28, 28,
    28, 29, 30, 30, 30, 31, 31, 32, 32, 32,
    33, 33, 34, 34, 35, 35, 35, 36, 36, 37,
    37, 37, 37, 38, 38, 39, 39, 39, 40, 40,
    41, 41, 42, 42, 43, 44, 44, 45, 45, 46,
    46, 47, 47, 48, 48, 49, 50, 50, 51, 52,
    52, 53, 54, 55, 56, 57, 57, 58, 59, 60,
    61, 62, 63, 64, 65, 66, 67, 67, 68, 70,
    71, 72, 72, 73, 74, 76, 77, 78, 79, 79,
    80, 83, 84, 86, 86, 87, 87, 88, 88, 90,
    90, 90, 95, 95, 95, 100, 100, 100};

// 1) Stock linear mapping: y = x.
inline double linearCurve(double joy) { return joy; }

// 2) Lookup-table mapping (values are percent 0-100, rescaled to 0-127).
inline double lookupCurve(double joy) {
    int idx = static_cast<int>(std::fabs(joy));
    if (idx > 127) idx = 127;
    const double pct = static_cast<double>(kTrueSpeed[static_cast<size_t>(idx)]);
    const double out = pct / 100.0 * kJoyMax;
    return joy < 0 ? -out : out;
}

// 3) Exponential curve: y = 1.2*(1.043)^x - 1.2 + 0.2x  (x >= 0, mirrored).
inline double exponentialCurve(double joy) {
    const double x = std::fabs(joy);
    const double y = 1.2 * std::pow(1.043, x) - 1.2 + 0.2 * x;
    const double out = std::min(y, kJoyMax);
    return joy < 0 ? -out : out;
}

// 4) Final S-curve: normalized logistic, odd-symmetric.
//    k controls midpoint steepness; 0.048 reproduces the Desmos shape:
//    gentle below ~25% stick, lively through the middle, saturating on top.
inline double sCurve(double joy, double k = 0.048) {
    const double x = std::fabs(joy);
    const double raw = 1.0 / (1.0 + std::exp(-k * (x - kJoyMax / 2.0)));
    const double lo = 1.0 / (1.0 + std::exp(k * kJoyMax / 2.0));
    const double hi = 1.0 / (1.0 + std::exp(-k * kJoyMax / 2.0));
    const double y = (raw - lo) / (hi - lo) * kJoyMax;  // normalize 0..127
    return joy < 0 ? -y : y;
}

}  // namespace vexlib
