# vex-odometry-pid

Autonomous motion stack from my VEX Robotics Competition seasons (teams
3303C / 3303D, later 3457C): **three-wheel field odometry**, a **PID motion
controller** with windup guards, settling logic, and timeouts, and the
**drive-curve** evolution (lookup table → exponential → S-curve) that made
our robots controllable at low speed.

> **Provenance — read this first.** The original source lived on a laptop I
> lost (ocean incident; data recovery pending). This repository is a
> reconstruction, rebuilt in July 2026 from the code and constants preserved
> in my engineering notebooks and on
> [my portfolio](https://anirudhsara35.github.io/Portfolio/). The verbatim
> notebook-era listings are kept unmodified in [`legacy/`](legacy/); the
> library in `include/` is the same math and control logic, cleaned up,
> made hardware-agnostic, and covered with tests so it can build and run
> anywhere — no robot required.

## What's here

```
include/
  pid.hpp           PID controller: windup guard, settling counter, timeout
  odometry.hpp      Three-tracking-wheel arc-chord odometry (Pilons method)
  drive_curves.hpp  Joystick mappings: linear, lookup table, exponential, S-curve
sim/
  simulator.hpp     Minimal differential-drive simulator (tracking-wheel model)
  demo.cpp          Drives a simulated square; odometry vs ground truth
tests/
  test_main.cpp     286 checks: odometry geometry, PID behavior, curves,
                    full-stack square-path integration test
legacy/
  notebook_code.md  The original competition listings, verbatim
docs/
  math.md           Arc-chord derivation, tuning process, and the two bugs
                    the original notebook listing contained
```

## Build and run

Requires only a C++17 compiler and `make`. No VEX SDK needed.

```
make run-tests   # builds and runs the test suite
make demo        # builds the simulator demo
./build/demo
```

Demo output — the full stack (PID driving, odometry tracking) around a
24-inch square in simulation:

```
after leg 4:  odom (0.00, -0.00, 360.0 deg)   truth (0.00, -0.00, 360.0 deg)
odometry-vs-truth drift after full square: 0.0000 in
```

## The three pieces

**PID** (`pid.hpp`). Built after stock VEX movement functions proved too
inaccurate to win autonomous. Tuned Ziegler–Nichols style: measured ultimate
gain Ku = 0.245 and oscillation period 0.675 s on the real robot, derived
starting constants, hand-tuned from there (drive: Kp 0.0675 / Ki 0.0005 /
Kd 0.3; turn: 0.7 / 0.0005 / 0.3). Two hard-won safety features: a timeout
so the loop can never hang mid-autonomous, and a settling counter so a move
only completes after the error stays small for consecutive checks.

**Odometry** (`odometry.hpp`). Three unpowered tracking wheels (offsets
SL = SR = 5", SS = 7.75", 2.75" wheels) reconstruct full (x, y, θ) between
updates using the arc-chord model from the
[Pilons tracking paper](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf):

    Δθ = (ΔL − ΔR) / (SL + SR)
    local offset = 2 sin(Δθ/2) · ⟨ΔS/Δθ + SS, ΔR/Δθ + SR⟩

rotated into the field frame by the average heading over the step. This
turned our autonomous routines from blind timed movements into instructions
in field coordinates.

**Drive curves** (`drive_curves.hpp`). Three generations of joystick
mapping: a hand-built 128-entry lookup table (3303D), the exponential curve
`y = 1.2(1.043)^x − 1.2 + 0.2x` (3303C), and the final S-curve — gentle near
zero for lining up shots, steep through the middle, flat at the top.

## Testing

`tests/test_main.cpp` verifies: straight-line and pure-rotation odometry
against closed-form answers, odometry against simulator ground truth on
arcs, PID convergence on a first-order plant, timeout firing on an
unreachable target, output clamping, curve symmetry/monotonicity/endpoints,
and an end-to-end square path where odometry must agree with ground truth
and the robot must return near its start.

## Results these systems produced

Used across VEX VRC and VEX AI seasons: Top 5 nationally / Top 12 worldwide
(VEX AI, 3457C), 8th on the VEX World Skills leaderboard, 20+ tournament
awards. Details on [my portfolio](https://anirudhsara35.github.io/Portfolio/).

## License

MIT — see [LICENSE](LICENSE).
