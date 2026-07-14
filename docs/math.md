# Math and tuning notes

## Arc-chord odometry

Between control-loop updates the robot travels along an arc of constant
curvature. With three unpowered tracking wheels — left and right at offsets
SL and SR from the tracking centre, a back (strafe) wheel at offset SS —
each loop reads three travel deltas ΔL, ΔR, ΔS (inches, converted from
encoder ticks by wheel circumference / ticks-per-rev).

**Heading change.** The left and right wheels ride arcs of radius
(r + SL) and (r − SR) around the same centre, so their travel difference
depends only on the rotation:

    Δθ = (ΔL − ΔR) / (SL + SR)

**Translation.** The arc's chord — the straight-line displacement — has
length 2 sin(Δθ/2) · radius. Applying this to the right and back wheels
(each corrected for its own offset from the tracking centre):

    local x (strafe)  = 2 sin(Δθ/2) · (ΔS/Δθ + SS)
    local y (forward) = 2 sin(Δθ/2) · (ΔR/Δθ + SR)

When Δθ = 0 the motion is a straight line and the local offset is simply
(ΔS, ΔR).

**Global frame.** The local offset is rotated by the average orientation
over the step, θ + Δθ/2 (heading clockwise-positive; at θ = 0 the robot
faces +y):

    x += local_y · sin(θ + Δθ/2) + local_x · cos(θ + Δθ/2)
    y += local_y · cos(θ + Δθ/2) − local_x · sin(θ + Δθ/2)
    θ += Δθ

Reference: [Pilons position tracking paper](http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf).

### Bugs in the original notebook listing

The listing preserved in `legacy/notebook_code.md` (and on my portfolio)
was an intermediate draft and contains two transcription bugs worth naming,
because finding them is a good code-review exercise:

1. `theta = atanh((deltaY/deltaX))` — hyperbolic arctangent where the polar
   conversion wants `atan2(deltaY, deltaX)`.
2. `deltaY = (2*sin(deltaTheta))*(...)` — missing the `/2`; both chord
   components use `2 sin(Δθ/2)`.

This rebuild implements the corrected math and pins it with unit tests
(pure rotation must produce zero translation; arcs must match the
simulator's ground truth).

## PID tuning process

1. Zero Ki and Kd; raise Kp until sustained oscillation.
   Measured on the competition robot: **Ku = 0.245, period Pu = 0.675 s**.
2. Derive starting constants (Ziegler–Nichols), then hand-tune:
   add Ki until steady-state error disappears, Kd until overshoot is gone.
3. Final competition gains: drive 0.0675 / 0.0005 / 0.3,
   turn 0.7 / 0.0005 / 0.3, loop period 15 ms.

Operational safeguards added after real failures:

- **Timeout** — an early controller looped forever when the robot stalled
  short of its target, freezing the autonomous routine.
- **Velocity cap** — for delicate movements (alignment, scoring).
- **Settling counter** — error must stay in tolerance for N consecutive
  checks (competition value 100) before a move reports done, so a single
  noisy encoder read can't end a move early.
- **Windup guard** — integral resets on zero-crossing/overshoot.

## Drive curves

Stock mapping is linear: joystick in [−127, 127] straight to velocity.
Problem: small inputs produce too much speed for precise alignment.

1. **Lookup table (3303D)** — 128 hand-built entries (`trueSpeed`,
   `halfSpeed`) smoothing acceleration for stall-prone motors.
2. **Exponential (3303C)** — `y = 1.2(1.043)^x − 1.2 + 0.2x`: fine control
   near zero, ramping to full speed.
3. **S-curve (final)** — found through Desmos iteration; implemented here
   as a normalized logistic `k = 0.048`. Gentle below ~25% stick, lively
   through the middle, saturating at full power. The equation replaced the
   entire lookup table with one line of math.
