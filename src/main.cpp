// Macros
#define stopDriveMotors(brakeMethod)  \
  MotorFR.setStopping(brakeMethod);   \
  MotorRR.setStopping(brakeMethod);   \
  MotorFL.setStopping(brakeMethod);   \
  MotorRL.setStopping(brakeMethod);   \
  MotorFR.stop();                     \
  MotorRR.stop();                     \
  MotorFL.stop();                     \
  MotorRL.stop()

#define spinDriveMotors(Direction)  \
  MotorFR.spin(Direction);        \
  MotorRR.spin(Direction);        \
  MotorFL.spin(Direction);        \
  MotorRL.spin(Direction)

#define rightSide(speed, unit)      \
  MotorFR.setVelocity(speed, unit); \
  MotorRR.setVelocity(speed, unit)

#define leftSide(speed, unit)       \
  MotorFL.setVelocity(speed, unit); \
  MotorRL.setVelocity(speed, unit)

#include "vex.h"
#include <string>
using namespace vex;
//competition Competition;

// Hardware
brain Brain;
controller Controller1 = controller(primary);

motor MotorFL = motor(PORT14, ratio18_1, true);
motor MotorRL = motor(PORT16, ratio18_1, true);
motor MotorFR = motor(PORT20, ratio18_1, false);
motor MotorRR = motor(PORT1, ratio18_1, false);

inertial Motion = inertial(PORT3);
motor_group LeftDriveSmart = motor_group(MotorFL, MotorRL);
motor_group RightDriveSmart = motor_group(MotorFR, MotorRR);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, Motion, 319.19, 320, 40, mm, 1);

digital_out Wings = digital_out(Brain.ThreeWirePort.A);

// Settings
const int MOTOR_TORQUE = 80;
  // Auton
  const int DRIVE_VELOCITY = 5;
  const int TURN_VELOCITY = 25;
  // Driver Control
  const double speedMultiplier = .3;
  const double spinMultiplier = .3;

// Global Variables
double leftSpeed = 0;
double rightSpeed = 0;

double desiredAngle = 0;

// Enumerations
enum directional {REVERSE = -1, FORWARD = 1};

// Constants
const double TILE_LENGTH = 8;
const int SCREENX = 480;
const int SCREENY = 272;
const int C_SCREENX = 20;


// Thread callbacks
void temperatureMonitor();

// Function declarations
void autonomous();
void driverControl();
void drive(directional direction, double dist);
void turn (int angle);


// MAIN FUNCTION HERE

int main() {
  Motion.calibrate();
  //MainMenu StartMenu;
  //StartMenu.run();
  while (Motion.isCalibrating()) {wait(5, msec);}
  thread TMON = thread(temperatureMonitor);
  //Competition.autonomous(autonomous);
  //Competition.drivercontrol(driverControl);
  
  autonomous();

  while (true) {
    wait(100, msec);
  }
}

void autonomous(void) {
  drive(FORWARD, 1);
}

void drive(directional direction, double dist)
{
  double initialPosition = MotorFL.position(rev);
  double initialHeading = Motion.heading();

  leftSide(DRIVE_VELOCITY, percent);
  rightSide(DRIVE_VELOCITY, percent);

  switch (direction) {
    case REVERSE:
      spinDriveMotors(reverse);
      break;
    
    case FORWARD:
      spinDriveMotors(forward);
      break;
  }

  while ((double)MotorFL.position(rev) != initialPosition + dist)
  {
    leftSide(DRIVE_VELOCITY + (Motion.heading() - initialHeading), percent);
    rightSide(DRIVE_VELOCITY - (Motion.heading() - initialHeading), percent);
  }
  stopDriveMotors(hold);
  wait(.2, sec);
}

void turn (int angle) {
  double desiredHeading = Motion.heading() + angle;
  while (desiredHeading > 360) {
    desiredHeading -= 360;
  }
  
  desiredAngle = desiredHeading;

  leftSide(TURN_VELOCITY, percent);
  rightSide(-1*TURN_VELOCITY, percent);
  spinDriveMotors(forward);

  while ((double)Motion.heading() != desiredHeading)
  {
    double sqrtdifference = log((desiredHeading - (Motion.heading()))+1);
    leftSide((TURN_VELOCITY/10) * sqrtdifference, percent);
    rightSide(-1*(TURN_VELOCITY/10) * sqrtdifference, percent);
  }
  stopDriveMotors(hold);
  wait(.2, sec);
}

void driverControl(void) {
  MotorFR.setMaxTorque(MOTOR_TORQUE, percent);
  MotorRR.setMaxTorque(MOTOR_TORQUE, percent);
  MotorFL.setMaxTorque(MOTOR_TORQUE-(MOTOR_TORQUE/5.5), percent);
  MotorRL.setMaxTorque(MOTOR_TORQUE-(MOTOR_TORQUE/5.5), percent);

  while (true) {
    // Drive Control
    // Setting variables
    leftSpeed = (Controller1.Axis3.position()*speedMultiplier) + (Controller1.Axis1.position()*spinMultiplier);
    rightSpeed = (Controller1.Axis3.position()*speedMultiplier) - (Controller1.Axis1.position()*spinMultiplier);
    // Applying Movement
    if (fabs(leftSpeed)+fabs(rightSpeed) > 5) {
      leftSide(leftSpeed, percent);
      rightSide(rightSpeed, percent);
      spinDriveMotors(forward);
    } else {
      stopDriveMotors(brake);
    }
  }
}

void temperatureMonitor() {
  double motorTempAvg = 0;
  Brain.Screen.setPenColor(black);
  Brain.Screen.setFillColor(black);
  Brain.Screen.drawRectangle(0, 0, SCREENX, SCREENY);
  while (true) {
    motorTempAvg = (MotorFR.temperature(percent) + MotorFL.temperature(percent)
                    + MotorRR.temperature(percent) + MotorRL.temperature(percent)) / 4;
    
    Brain.Screen.setFont(mono40);
    Brain.Screen.setPenColor(white);

    // Printing all motor temperatures & average
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%d", (int)(MotorFR.temperature(percent)));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("%d", (int)(MotorFL.temperature(percent)));
    Brain.Screen.setCursor(1, 23);
    Brain.Screen.print("%d", (int)(MotorRR.temperature(percent)));
    Brain.Screen.setCursor(7, 23);
    Brain.Screen.print("%d", (int)(MotorRL.temperature(percent)));
    Brain.Screen.setCursor(3, 7);
    Brain.Screen.print("MotorAVG: %d", (int)(motorTempAvg));

    // Printing Heading
    Brain.Screen.setCursor(3, 1);
    Brain.Screen.print("%d", (int)(Motion.heading()));
    Brain.Screen.setCursor(4, 1);
    Brain.Screen.print("%d", (int)(desiredAngle));
  }
}

class MainMenu {
public:
  void run() {
    
  }

  MainMenu() {
    /* // Competition Button
    CompetitionButton.text = "Competition Mode";
    CompetitionButton.menu = COMPETITION;

    // Driver Control Button
    DCButton.text = "Driver Control";
    DCButton.menu = DRIVER_CONTROL;

    // Autonomous Button
    AutonButton.text = "Autonomous";
    AutonButton.menu = AUTONOMOUS; */
  }

private:

  /* enum menuLink {COMPETITION, DRIVER_CONTROL, AUTONOMOUS};

  struct menuButton {
    char *text;
    menuLink menu{};
  };

  // Buttons
  menuButton CompetitionButton;
  menuButton DCButton;
  menuButton AutonButton; */
};