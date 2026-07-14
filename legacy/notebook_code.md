# Original notebook listings (verbatim)

These are the competition-era listings exactly as preserved in my
engineering notebooks and on
[my portfolio](https://anirudhsara35.github.io/Portfolio/), including their
bugs and rough edges (see `docs/math.md` for the two known bugs in the
odometry draft). They target VEXcode / PROS / OkapiLib and require robot
hardware; the buildable, corrected library lives in `include/`.

## First PID controller (3303C — inertial turn)

```cpp
int turnPID(){
  while(enableTurnPID){
    int heading = inertial.get_heading();
    desiredTurnValue -= heading;
    error = desiredTurnValue - heading;
    derivative = error - prevError;
    totalError += error;
    prevError = error;
    double turnPower = (error*kP + derivative*kD + totalError*kI);
    rightBack.moveVoltage(-turnPower);
    rightMid.moveVoltage(-turnPower);
    rightTop.moveVoltage(-turnPower);
    leftBack.moveVoltage(turnPower);
    leftMid.moveVoltage(turnPower);
    leftTop.moveVoltage(turnPower);
    std::string HEADING = std::to_string(heading);
    pros::lcd::set_text(1, HEADING);
    pros::delay(20);
  }
  return 1;
}
```

## Okapi rebuild with timeout + velocity cap

```cpp
void driveForward(double distance, int ms, double maxVelocity) {
  IterativePosPIDController drivePID =
      IterativeControllerFactory::posPID(0.0014, 0.0002, 0.00015);
  drivePID.setTarget(distance);
  double initLeftPos = leftMid.getPosition();
  double initRightPos = rightMid.getPosition();
  int timer = 0;
  while (timer < ms) {
    double currentLeftPos = leftMid.getPosition();
    double currentRightPos = rightMid.getPosition();
    double vel = drivePID.step((currentLeftPos + currentRightPos) / 2);
    if (vel > maxVelocity){ vel = maxVelocity; }
    drive -> getModel() -> tank(vel, vel);
    pros::delay(10);
    timer += 10;
  }
  rightTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightMid.setBrakeMode(AbstractMotor::brakeMode::hold);
  rightBack.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftTop.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftMid.setBrakeMode(AbstractMotor::brakeMode::hold);
  leftBack.setBrakeMode(AbstractMotor::brakeMode::hold);
  drive -> getModel() -> tank(0, 0);
}

void moveF(double dist, int m, double maxV) // inches
{
  tareEncoders();
  driveForward((dist * 1057), m, maxV);  // 1057 ticks per inch
}
```

## The 3303D PID class

```cpp
class pidController{
public:
  double Kp = 0.0675;
  double Ki = 0.0005;
  double Kd = 0.3;
  double turnKp = 0.7;
  double turnKi = 0.0005;
  double turnKd = 0.3;
  int Dt = 15;
  double error, speed, integral, derivative, prevError;
  void pidDriveLoop(int setpoint);
  void pidDriveBackLoop(int setpoint);
  void pidTurnRightLoop(int setpoint);
  void pidTurnLeftLoop(int setpoint);
  bool pidRun;
};

void pidController::pidDriveLoop(int setpoint){
  mainBaseLeftEncReset;
  mainBaseRightEncReset;
  bool moveComplete = false;
  int timesGood = 0;
  integral = 0; derivative = 0; prevError = 0;
  while(!moveComplete && mainBaseEnc <= setpoint){
    error = setpoint - mainBaseEnc;
    integral = integral + error;
    if(error <= 0 || mainBaseEnc > setpoint){ integral = 0; }
    if(error > setpoint){ integral = 0; }
    derivative = error - prevError;
    prevError = error;
    speed = error*Kp + integral*Ki + derivative*Kd;
    vex::task::sleep(Dt);
    LB.spin(fwd,speed,pct); RB.spin(fwd,speed,pct);
    LF.spin(fwd,speed,pct); RF.spin(fwd,speed,pct);
    LT.spin(fwd,speed,pct); RT.spin(fwd,speed,pct);
    if(error <= 100){ timesGood++; }
    if(timesGood >= 100){ moveComplete = true; }
    task::sleep(1);
  }
  LF.stop(vex::brakeType::brake); RB.stop(vex::brakeType::brake);
  LT.stop(vex::brakeType::brake); RT.stop(vex::brakeType::brake);
  LB.stop(vex::brakeType::brake); RF.stop(vex::brakeType::brake);
}
```

## Odometry tracking loop (draft — contains the two documented bugs)

```cpp
#include "subsystems/drive.hpp"
// http://thepilons.ca/wp-content/uploads/2018/10/Tracking.pdf
#define Pi 3.14159265358979323846
#define SL 5      // tracking centre to middle of left wheel
#define SR 5      // tracking centre to middle of right wheel
#define SS 7.75   // tracking centre to middle of back wheel
#define WheelDiam 2.75
#define wheelCircum WheelDiam*Pi

double deltaL, deltaR, deltaS, deltaTheta,
       currentR, currentL, currentS,
       previousR, previousL, previousS, deltaX, deltaY;
double theta = 0, thetaR = 0, newTheta = 0, thetaM = 0;
double x = 0, y = 0, angle = 0, thetaInertial = 0;
bool enableOdom = true;

pros::ADIEncoder rightTracking ('A','B', false);
pros::ADIEncoder leftTracking ('C','D', true);
pros::ADIEncoder backTracking ('E','F', false);

int trackPos(){
  while(enableOdom){
    // current encoder values
    currentR = rightTracking.get_value();
    currentL = leftTracking.get_value();
    currentS = backTracking.get_value();
    // change in distance on each side
    deltaR = ((currentR - previousR) * wheelCircum)/360;
    deltaL = ((currentL - previousL) * wheelCircum)/360;
    deltaS = ((currentS - previousS) * wheelCircum)/360;
    previousR = currentR; previousL = currentL; previousS = currentS;
    // change in angle
    newTheta = (thetaR + (deltaL - deltaR)/(SL + SR));
    deltaTheta = newTheta - angle;
    if (deltaTheta == 0){ // straight line
      deltaX = deltaS;
      deltaY = deltaR;
    }
    else { // arc motion
      deltaX = (2*sin(deltaTheta/2))*(deltaS/deltaTheta + SS);
      deltaY = (2*sin(deltaTheta))*(deltaR/deltaTheta + SR);   // BUG: should be sin(deltaTheta/2)
    }
    thetaM = angle + deltaTheta/2; // average orientation
    theta = atanh((deltaY/deltaX));                            // BUG: should be atan2(deltaY, deltaX)
    double radius = sqrt(deltaX*deltaX + deltaY*deltaY);
    theta = theta - thetaM;
    deltaX = radius*cos(theta);
    deltaY = radius*sin(theta);
    thetaInertial = inertial.get_heading();
    x = x + deltaX;
    y = y + deltaY;
    pros::lcd::set_text(1, "X: " + std::to_string(x));
    pros::lcd::set_text(2, "Y: " + std::to_string(y));
    pros::lcd::set_text(3, "Theta: " + std::to_string(theta));
  }
  return 1;
}
```
