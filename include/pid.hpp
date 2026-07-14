#pragma once
// pid.hpp — Hardware-agnostic PID controller
//
// Reconstruction of the PID motion-control library I built for VEX teams
// 3303C / 3303D (original source lost with my laptop; rebuilt from my
// engineering-notebook code preserved on my portfolio site).
//
// Design notes carried over from the competition version:
//   * Integral windup guard: integral resets when the error crosses zero
//     or the mechanism overshoots the setpoint.
//   * Settling counter: a move is only "done" after the error stays inside
//     tolerance for `settleCount` consecutive steps (competition value: 100).
//   * Timeout: the loop always exits, even if the target is never reached.
//     (Added after a frozen robot cost us an autonomous period.)
//
// Original tuning method: Ziegler–Nichols-style. Measured ultimate gain
// Ku = 0.245 and oscillation period Pu = 0.675 s, derived starting
// constants, then hand-tuned on the field.

#include <algorithm>
#include <cmath>

namespace vexlib {

struct PidGains {
    double kP = 0.0;
    double kI = 0.0;
    double kD = 0.0;
};

// Competition-tuned gains from the 3303D notebook.
inline constexpr double kDriveKp = 0.0675;
inline constexpr double kDriveKi = 0.0005;
inline constexpr double kDriveKd = 0.3;
inline constexpr double kTurnKp  = 0.7;
inline constexpr double kTurnKi  = 0.0005;
inline constexpr double kTurnKd  = 0.3;

class PidController {
public:
    PidController(PidGains gains, double dtSeconds,
                  double settleTolerance, int settleCount,
                  double timeoutSeconds, double outputLimit)
        : gains_(gains),
          dt_(dtSeconds),
          settleTolerance_(settleTolerance),
          settleCount_(settleCount),
          timeout_(timeoutSeconds),
          outputLimit_(outputLimit) {}

    void setTarget(double target) {
        target_ = target;
        reset();
    }

    double target() const { return target_; }

    // Advance one control step. `measurement` is the current process value.
    // Returns the commanded output, clamped to [-outputLimit, outputLimit].
    double step(double measurement) {
        const double error = target_ - measurement;

        // Integral windup guards (same policy as the notebook code):
        // reset accumulated error on zero-crossing or overshoot.
        if ((error <= 0.0 && prevError_ > 0.0) ||
            (error >= 0.0 && prevError_ < 0.0)) {
            integral_ = 0.0;
        }
        integral_ += error * dt_;

        const double derivative = (error - prevError_) / dt_;
        prevError_ = error;

        double output = error * gains_.kP + integral_ * gains_.kI +
                        derivative * gains_.kD;
        output = std::clamp(output, -outputLimit_, outputLimit_);

        // Settling logic: done only after `settleCount_` consecutive
        // in-tolerance steps.
        if (std::fabs(error) <= settleTolerance_) {
            ++timesGood_;
        } else {
            timesGood_ = 0;
        }
        elapsed_ += dt_;
        return output;
    }

    bool settled() const { return timesGood_ >= settleCount_; }
    bool timedOut() const { return elapsed_ >= timeout_; }
    bool done() const { return settled() || timedOut(); }

    double lastError() const { return prevError_; }

    void reset() {
        integral_ = 0.0;
        prevError_ = 0.0;
        timesGood_ = 0;
        elapsed_ = 0.0;
    }

private:
    PidGains gains_;
    double dt_;
    double settleTolerance_;
    int settleCount_;
    double timeout_;
    double outputLimit_;

    double target_ = 0.0;
    double integral_ = 0.0;
    double prevError_ = 0.0;
    int timesGood_ = 0;
    double elapsed_ = 0.0;
};

}  // namespace vexlib
