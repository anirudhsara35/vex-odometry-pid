#pragma once
// odometry.hpp — Three-tracking-wheel odometry (arc-chord method)
//
// Reconstruction of the field-odometry system I built for VEX teams
// 3303C / 3303D, based on the Pilons tracking paper:
//   http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
//
// Geometry (competition robot, inches):
//   SL = 5.00   tracking centre -> left wheel
//   SR = 5.00   tracking centre -> right wheel
//   SS = 7.75   tracking centre -> back (strafe) wheel
//   tracking wheel diameter = 2.75
//
// Core idea: between updates the robot travels along an arc. The three
// encoder deltas give the change in heading; the chord of the arc gives
// the true local translation:
//
//   dtheta   = (dL - dR) / (SL + SR)
//   local dx = 2 sin(dtheta/2) * (dS/dtheta + SS)
//   local dy = 2 sin(dtheta/2) * (dR/dtheta + SR)
//
// The local offset is then rotated into the global frame using the
// average orientation over the step (theta + dtheta/2).
//
// NOTE: this rebuild fixes two transcription bugs that lived in the
// original notebook listing (documented in docs/math.md): the original
// used atanh() where atan2() was intended, and dropped the /2 in one
// 2*sin(dtheta/2) term. The competition robot ran the corrected math;
// the notebook page preserved the earlier draft.

#include <cmath>

namespace vexlib {

struct Pose {
    double x = 0.0;      // inches, field frame
    double y = 0.0;      // inches, field frame
    double theta = 0.0;  // radians, CCW positive
};

struct OdometryGeometry {
    double sL = 5.0;             // in
    double sR = 5.0;             // in
    double sS = 7.75;            // in
    double wheelDiameter = 2.75; // in
    double ticksPerRev = 360.0;  // optical shaft encoder
};

class ThreeWheelOdometry {
public:
    explicit ThreeWheelOdometry(OdometryGeometry geom = {}) : geom_(geom) {}

    void setPose(const Pose& pose) { pose_ = pose; }
    const Pose& pose() const { return pose_; }

    double ticksToInches(double ticks) const {
        const double circumference = geom_.wheelDiameter * M_PI;
        return ticks / geom_.ticksPerRev * circumference;
    }

    // Update from absolute encoder readings (ticks), as read each loop
    // from the left, right, and back tracking wheels.
    void updateFromTicks(double leftTicks, double rightTicks,
                         double backTicks) {
        const double dL = ticksToInches(leftTicks - prevLeft_);
        const double dR = ticksToInches(rightTicks - prevRight_);
        const double dS = ticksToInches(backTicks - prevBack_);
        prevLeft_ = leftTicks;
        prevRight_ = rightTicks;
        prevBack_ = backTicks;
        updateFromDistances(dL, dR, dS);
    }

    // Update from per-step wheel travel distances (inches).
    void updateFromDistances(double dL, double dR, double dS) {
        const double dTheta = (dL - dR) / (geom_.sL + geom_.sR);

        double localX;  // strafe (robot-right positive)
        double localY;  // forward
        if (std::fabs(dTheta) < 1e-9) {
            // Straight line: chord == arc.
            localX = dS;
            localY = dR;
        } else {
            // Arc-chord: 2 sin(dtheta/2) * (arc/dtheta + offset)
            const double chordScale = 2.0 * std::sin(dTheta / 2.0);
            localX = chordScale * (dS / dTheta + geom_.sS);
            localY = chordScale * (dR / dTheta + geom_.sR);
        }

        // Rotate the local offset by the average heading over the step.
        const double avgTheta = pose_.theta + dTheta / 2.0;
        const double cosA = std::cos(avgTheta);
        const double sinA = std::sin(avgTheta);

        pose_.x += localY * sinA + localX * cosA;
        pose_.y += localY * cosA - localX * sinA;
        pose_.theta += dTheta;
    }

private:
    OdometryGeometry geom_;
    Pose pose_{};
    double prevLeft_ = 0.0;
    double prevRight_ = 0.0;
    double prevBack_ = 0.0;
};

}  // namespace vexlib
