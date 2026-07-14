#pragma once
// simulator.hpp — Minimal differential-drive simulator
//
// Lets the odometry + PID stack run and be tested without robot hardware.
// Conventions match the library: heading is clockwise-positive; at
// theta = 0 the robot faces +y; forward travel dF maps to
// (dx, dy) = (dF sin(theta), dF cos(theta)).
//
// Tracking-wheel model (tank drive, no strafe):
//   dL = (v + w*SL) * dt        left wheel, offset SL left of centre
//   dR = (v - w*SR) * dt        right wheel, offset SR right of centre
//   dS = -w * SS * dt           back wheel, offset SS behind centre

#include <cmath>

#include "../include/odometry.hpp"

namespace vexsim {

class DiffDriveSim {
public:
    explicit DiffDriveSim(vexlib::OdometryGeometry geom = {},
                          double trackWidth = 12.0)
        : geom_(geom), trackWidth_(trackWidth) {}

    // Advance the true robot state by dt with left/right wheel speeds
    // (inches per second). Returns the tracking-wheel deltas (inches).
    struct WheelDeltas {
        double dL, dR, dS;
    };

    WheelDeltas step(double vLeft, double vRight, double dt) {
        const double v = (vLeft + vRight) / 2.0;              // forward
        const double w = (vLeft - vRight) / trackWidth_;      // CW-positive

        pose_.x += v * dt * std::sin(pose_.theta);
        pose_.y += v * dt * std::cos(pose_.theta);
        pose_.theta += w * dt;

        return WheelDeltas{(v + w * geom_.sL) * dt,
                           (v - w * geom_.sR) * dt,
                           -w * geom_.sS * dt};
    }

    const vexlib::Pose& truePose() const { return pose_; }
    void setTruePose(const vexlib::Pose& p) { pose_ = p; }

private:
    vexlib::OdometryGeometry geom_;
    double trackWidth_;
    vexlib::Pose pose_{};
};

}  // namespace vexsim
