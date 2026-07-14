// test_main.cpp — Unit tests for the PID / odometry / drive-curve library.
// No external test framework; plain asserts with a pass counter.
//
// Build & run:  make test && ./build/tests

#include <cmath>
#include <cstdio>
#include <cstdlib>

#include "../include/drive_curves.hpp"
#include "../include/odometry.hpp"
#include "../include/pid.hpp"
#include "../sim/simulator.hpp"

using vexlib::PidController;
using vexlib::PidGains;
using vexlib::Pose;
using vexlib::ThreeWheelOdometry;
using vexsim::DiffDriveSim;

static int gChecks = 0;
static int gFailures = 0;

#define CHECK_NEAR(actual, expected, tol, msg)                             \
    do {                                                                   \
        ++gChecks;                                                         \
        const double a_ = (actual);                                        \
        const double e_ = (expected);                                      \
        if (std::fabs(a_ - e_) > (tol)) {                                  \
            ++gFailures;                                                   \
            std::printf("FAIL %s: got %.6f, expected %.6f (tol %.6f)\n",   \
                        msg, a_, e_, (double)(tol));                       \
        }                                                                  \
    } while (0)

#define CHECK_TRUE(cond, msg)                                              \
    do {                                                                   \
        ++gChecks;                                                         \
        if (!(cond)) {                                                     \
            ++gFailures;                                                   \
            std::printf("FAIL %s\n", msg);                                 \
        }                                                                  \
    } while (0)

// ---------------------------------------------------------------- odometry

static void testOdometryStraightLine() {
    ThreeWheelOdometry odom;
    for (int i = 0; i < 240; ++i)
        odom.updateFromDistances(0.1, 0.1, 0.0);  // 24" forward
    CHECK_NEAR(odom.pose().x, 0.0, 1e-6, "straight: x");
    CHECK_NEAR(odom.pose().y, 24.0, 1e-6, "straight: y");
    CHECK_NEAR(odom.pose().theta, 0.0, 1e-9, "straight: theta");
}

static void testOdometryPureRotation() {
    ThreeWheelOdometry odom;
    const double target = M_PI / 2.0;  // 90 deg CW
    const int steps = 500;
    const double dTheta = target / steps;
    for (int i = 0; i < steps; ++i)
        odom.updateFromDistances(dTheta * 5.0, -dTheta * 5.0,
                                 -dTheta * 7.75);
    CHECK_NEAR(odom.pose().theta, target, 1e-6, "rotation: theta");
    CHECK_NEAR(odom.pose().x, 0.0, 1e-6, "rotation: x stays 0");
    CHECK_NEAR(odom.pose().y, 0.0, 1e-6, "rotation: y stays 0");
}

static void testOdometryMatchesSimulatorOnArc() {
    // Constant forward + turn rate for 2 s: robot sweeps an arc.
    DiffDriveSim sim;
    ThreeWheelOdometry odom;
    const double dt = 0.01;
    for (int i = 0; i < 200; ++i) {
        const auto d = sim.step(14.0, 10.0, dt);  // gentle right arc
        odom.updateFromDistances(d.dL, d.dR, d.dS);
    }
    CHECK_NEAR(odom.pose().x, sim.truePose().x, 0.05, "arc: x vs truth");
    CHECK_NEAR(odom.pose().y, sim.truePose().y, 0.05, "arc: y vs truth");
    CHECK_NEAR(odom.pose().theta, sim.truePose().theta, 1e-3,
               "arc: theta vs truth");
}

static void testOdometryTickInterface() {
    // 360 ticks = one wheel rev = one circumference of travel.
    ThreeWheelOdometry odom;
    odom.updateFromTicks(360.0, 360.0, 0.0);
    CHECK_NEAR(odom.pose().y, 2.75 * M_PI, 1e-9, "ticks: one rev forward");
}

// --------------------------------------------------------------------- pid

static void testPidConvergesOnFirstOrderPlant() {
    // Plant: velocity follows commanded output through a lag.
    PidController pid(PidGains{2.0, 0.1, 0.05}, 0.01, 0.05, 20, 10.0, 50.0);
    pid.setTarget(24.0);
    double position = 0.0, velocity = 0.0;
    while (!pid.done()) {
        const double u = pid.step(position);
        velocity += (u - velocity) * 0.2;  // first-order lag
        position += velocity * 0.01;
    }
    CHECK_TRUE(pid.settled(), "pid settles on reachable target");
    CHECK_NEAR(position, 24.0, 0.2, "pid final position");
}

static void testPidTimeoutOnUnreachableTarget() {
    // Plant never moves: settling is impossible, timeout must fire.
    PidController pid(PidGains{1.0, 0.0, 0.0}, 0.01, 0.05, 20, 0.5, 50.0);
    pid.setTarget(100.0);
    int iterations = 0;
    while (!pid.done() && iterations < 100000) {
        pid.step(0.0);
        ++iterations;
    }
    CHECK_TRUE(pid.timedOut(), "pid times out instead of hanging");
    CHECK_TRUE(iterations <= 51, "timeout fires at the configured time");
}

static void testPidOutputClamped() {
    PidController pid(PidGains{100.0, 0.0, 0.0}, 0.01, 0.05, 20, 1.0, 40.0);
    pid.setTarget(1000.0);
    CHECK_NEAR(pid.step(0.0), 40.0, 1e-9, "output clamped to limit");
}

// ------------------------------------------------------------ drive curves

static void testCurveSymmetryAndEndpoints() {
    for (double j : {5.0, 40.0, 90.0, 127.0}) {
        CHECK_NEAR(vexlib::sCurve(-j), -vexlib::sCurve(j), 1e-9,
                   "s-curve odd symmetry");
        CHECK_NEAR(vexlib::exponentialCurve(-j), -vexlib::exponentialCurve(j),
                   1e-9, "exponential odd symmetry");
    }
    CHECK_NEAR(vexlib::sCurve(0.0), 0.0, 1e-9, "s-curve zero at zero");
    CHECK_NEAR(vexlib::sCurve(127.0), 127.0, 1e-6, "s-curve full at full");
    CHECK_NEAR(vexlib::lookupCurve(127.0), 127.0, 1e-9, "lookup full at full");
}

static void testCurvesMonotonic() {
    double prevS = -1.0, prevE = -1.0;
    for (int j = 0; j <= 127; ++j) {
        const double s = vexlib::sCurve(j);
        const double e = vexlib::exponentialCurve(j);
        CHECK_TRUE(s >= prevS, "s-curve monotonic");
        CHECK_TRUE(e >= prevE, "exponential monotonic");
        prevS = s;
        prevE = e;
    }
}

static void testSCurveGentleAtLowStick() {
    // The whole point: less output than linear at low stick, similar at top.
    CHECK_TRUE(vexlib::sCurve(20.0) < 20.0, "s-curve gentle at low stick");
    CHECK_TRUE(vexlib::sCurve(115.0) > 100.0, "s-curve strong near full");
}

// ---------------------------------------------------------- integration

static void testSquarePathIntegration() {
    // Full stack: PID drives a simulated square; odometry must agree with
    // ground truth and end near the start.
    DiffDriveSim sim;
    ThreeWheelOdometry odom;
    const double dt = 0.015;

    for (int leg = 0; leg < 4; ++leg) {
        PidController drive(PidGains{6.0, 0.4, 0.8}, dt, 0.25, 10, 5.0, 40.0);
        drive.setTarget(24.0);
        double traveled = 0.0;
        while (!drive.done()) {
            const double v = drive.step(traveled);
            const auto d = sim.step(v, v, dt);
            traveled += (d.dL + d.dR) / 2.0;
            odom.updateFromDistances(d.dL, d.dR, d.dS);
        }
        PidController turn(PidGains{30.0, 1.0, 2.0}, dt, 0.005, 10, 5.0, 30.0);
        turn.setTarget(odom.pose().theta + M_PI / 2.0);
        while (!turn.done()) {
            const double v = turn.step(odom.pose().theta);
            const auto d = sim.step(v, -v, dt);
            odom.updateFromDistances(d.dL, d.dR, d.dS);
        }
    }
    const Pose& o = odom.pose();
    const Pose& t = sim.truePose();
    const double driftFromTruth =
        std::sqrt((o.x - t.x) * (o.x - t.x) + (o.y - t.y) * (o.y - t.y));
    CHECK_TRUE(driftFromTruth < 0.1, "square: odometry tracks ground truth");
    const double distFromStart = std::sqrt(o.x * o.x + o.y * o.y);
    CHECK_TRUE(distFromStart < 2.0, "square: robot returns near start");
}

int main() {
    testOdometryStraightLine();
    testOdometryPureRotation();
    testOdometryMatchesSimulatorOnArc();
    testOdometryTickInterface();
    testPidConvergesOnFirstOrderPlant();
    testPidTimeoutOnUnreachableTarget();
    testPidOutputClamped();
    testCurveSymmetryAndEndpoints();
    testCurvesMonotonic();
    testSCurveGentleAtLowStick();
    testSquarePathIntegration();

    std::printf("%d checks, %d failures\n", gChecks, gFailures);
    return gFailures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
}
