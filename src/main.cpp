// Macros
#define stopDriveMotors(brakeMethod)  \
  rightMotorA.setStopping(brakeMethod);   \
  rightMotorB.setStopping(brakeMethod);   \
  leftMotorA.setStopping(brakeMethod);   \
  leftMotorB.setStopping(brakeMethod);   \
  rightMotorA.stop();                     \
  rightMotorB.stop();                     \
  leftMotorA.stop();                     \
  leftMotorB.stop()

#define spinDriveMotors(Direction)  \
  rightMotorA.spin(Direction);        \
  rightMotorB.spin(Direction);        \
  leftMotorA.spin(Direction);        \
  leftMotorB.spin(Direction)

#define rightSide(speed, unit)      \
  rightMotorA.setVelocity(speed, unit); \
  rightMotorB.setVelocity(speed, unit)

#define leftSide(speed, unit)       \
  leftMotorA.setVelocity(speed, unit); \
  leftMotorB.setVelocity(speed, unit)

#include "vex.h"
using namespace vex;
competition Competition;
brain Brain;

// Hardware
controller Controller1 = controller(primary);

motor leftMotorA = motor(PORT20, ratio18_1, false);
motor leftMotorB = motor(PORT14, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT1, ratio18_1, true);
motor rightMotorB = motor(PORT16, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
inertial DrivetrainInertial = inertial(PORT3);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, DrivetrainInertial, 319.19, 320, 40, mm, 1);

digital_out Wings = digital_out(Brain.ThreeWirePort.A);

/* class MainMenu {
public:
  void run() {
    
  }

  MainMenu() {
    // Competition Button
    CompetitionButton.text = "Competition Mode";
    CompetitionButton.menu = COMPETITION;

    // Driver Control Button
    DCButton.text = "Driver Control";
    DCButton.menu = DRIVER_CONTROL;

    // Autonomous Button
    AutonButton.text = "Autonomous";
    AutonButton.menu = AUTONOMOUS;
  }

private:

  enum menuLink {COMPETITION, DRIVER_CONTROL, AUTONOMOUS};

  struct menuButton {
    char *text;
    menuLink menu{};
  };

  // Buttons
  menuButton CompetitionButton;
  menuButton DCButton;
  menuButton AutonButton;
}; */

void setBackground(const color &c) {
  Brain.Screen.setPenColor(c);
  Brain.Screen.setFillColor(c);
  Brain.Screen.drawRectangle(0, 0, SCREENX, SCREENY);
}

void pre_auton(void) {
  Drivetrain.setDriveVelocity(10, percent);
  Drivetrain.setTurnVelocity(5, percent);
}

void autonomous(void) {
  
}

void driverControl(void) {
  rightMotorA.setMaxTorque(MOTOR_TORQUE, percent);
  rightMotorB.setMaxTorque(MOTOR_TORQUE, percent);
  leftMotorA.setMaxTorque(MOTOR_TORQUE-(MOTOR_TORQUE/5.5), percent);
  leftMotorB.setMaxTorque(MOTOR_TORQUE-(MOTOR_TORQUE/5.5), percent);

  while (true) {
    // Drive Control
    // Setting variables
    leftSpeed = (Controller1.Axis3.position()*speedMult) + (Controller1.Axis1.position()*turnMult);
    rightSpeed = (Controller1.Axis3.position()*speedMult) - (Controller1.Axis1.position()*turnMult);
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

// Settings
const int MOTOR_TORQUE = 80;
const double speedMult = .3;
const double turnMult = .3;

// Global Variables
double leftSpeed = 0;
double rightSpeed = 0;

// Constants
const double TILE_LENGTH = 8;
const int TURN_ERROR = -6;
const int SCREENX = 480;
const int SCREENY = 272;

// Functions
void temperatureMonitor();

int main() {
  thread TMON = thread(temperatureMonitor);
  Competition.autonomous(autonomous);
  Competition.drivercontrol(driverControl);
  
  pre_auton();

  while (true) {
    wait(100, msec);
  }
}

void temperatureMonitor() {
  double motorTempAvg = 0;
  Brain.Screen.setPenColor(black);
  Brain.Screen.setFillColor(black);
  Brain.Screen.drawRectangle(0, 0, SCREENX, SCREENY);
  while (true) {
    motorTempAvg = (rightMotorA.temperature(percent) + leftMotorA.temperature(percent)
                    + rightMotorB.temperature(percent) + leftMotorB.temperature(percent)) / 4; // Adds up & divides all temperatures for average
    
    Brain.Screen.setFont(mono40);
    Brain.Screen.setPenColor(white);

    // Printing all motor temperatures & average
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%d", static_cast<int>(rightMotorA.temperature(percent)));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("%d", static_cast<int>(leftMotorA.temperature(percent)));
    Brain.Screen.setCursor(1, 23);
    Brain.Screen.print("%d", static_cast<int>(rightMotorB.temperature(percent)));
    Brain.Screen.setCursor(9, 23);
    Brain.Screen.print("%d", static_cast<int>(leftMotorB.temperature(percent)));
    Brain.Screen.setCursor(3, 7);
    Brain.Screen.print("MotorAVG: %d", static_cast<int>(motorTempAvg));

    if (Controller1.ButtonB.pressing()) {vexSystemExitRequest();}
  }
}