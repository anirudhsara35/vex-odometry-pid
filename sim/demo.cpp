// demo.cpp — Drive a simulated robot through waypoints using the
// PID + odometry stack, then compare odometry pose to ground truth.
//
// Build & run:  make demo && ./build/demo

#include <cstdio>

#include "../include/drive_curves.hpp"
#include "../include/odometry.hpp"
#include "../include/pid.hpp"
#include "simulator.hpp"

using vexlib::PidController;
using vexlib::PidGains;
using vexlib::Pose;
using vexlib::ThreeWheelOdometry;
using vexsim::DiffDriveSim;

namespace {

constexpr double kDt = 0.015;  // 15 ms loop, same as the competition code

// Drive straight a given distance (inches) under PID, updating odometry.
void driveDistance(DiffDriveSim& sim, ThreeWheelOdometry& odom,
                   double inches) {
    PidController pid(PidGains{6.0, 0.4, 0.8}, kDt,
                      /*settleTolerance=*/0.25, /*settleCount=*/10,
                      /*timeoutSeconds=*/5.0, /*outputLimit=*/40.0);
    pid.setTarget(inches);
    double traveled = 0.0;
    while (!pid.done()) {
        const double v = pid.step(traveled);
        const auto d = sim.step(v, v, kDt);
        traveled += (d.dL + d.dR) / 2.0;
        odom.updateFromDistances(d.dL, d.dR, d.dS);
    }
}

// Turn in place by a given angle (radians, CW-positive) under PID.
void turnAngle(DiffDriveSim& sim, ThreeWheelOdometry& odom, double radians) {
    PidController pid(PidGains{30.0, 1.0, 2.0}, kDt,
                      /*settleTolerance=*/0.005, /*settleCount=*/10,
                      /*timeoutSeconds=*/5.0, /*outputLimit=*/30.0);
    const double start = odom.pose().theta;
    pid.setTarget(start + radians);
    while (!pid.done()) {
        const double v = pid.step(odom.pose().theta);
        const auto d = sim.step(v, -v, kDt);
        odom.updateFromDistances(d.dL, d.dR, d.dS);
    }
}

void report(const char* label, const ThreeWheelOdometry& odom,
            const DiffDriveSim& sim) {
    const Pose& o = odom.pose();
    const Pose& t = sim.truePose();
    std::printf("%-28s odom (%7.2f, %7.2f, %6.1f deg)   "
                "truth (%7.2f, %7.2f, %6.1f deg)\n",
                label, o.x, o.y, o.theta * 180.0 / M_PI, t.x, t.y,
                t.theta * 180.0 / M_PI);
}

}  // namespace

int main() {
    DiffDriveSim sim;
    ThreeWheelOdometry odom;

    std::printf("Square path: 24\" forward + 90 deg CW turn, four times.\n\n");
    for (int leg = 1; leg <= 4; ++leg) {
        driveDistance(sim, odom, 24.0);
        turnAngle(sim, odom, M_PI / 2.0);
        char label[32];
        std::snprintf(label, sizeof(label), "after leg %d:", leg);
        report(label, odom, sim);
    }

    const Pose& o = odom.pose();
    const Pose& t = sim.truePose();
    const double drift =
        std::sqrt((o.x - t.x) * (o.x - t.x) + (o.y - t.y) * (o.y - t.y));
    std::printf("\nodometry-vs-truth drift after full square: %.4f in\n",
                drift);

    std::printf("\nDrive-curve comparison at joystick = 20 / 64 / 127:\n");
    for (double j : {20.0, 64.0, 127.0}) {
        std::printf("  joy %5.0f  linear %6.1f  lookup %6.1f  "
                    "exponential %6.1f  s-curve %6.1f\n",
                    j, vexlib::linearCurve(j), vexlib::lookupCurve(j),
                    vexlib::exponentialCurve(j), vexlib::sCurve(j));
    }
    return 0;
}
