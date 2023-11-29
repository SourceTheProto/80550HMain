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
using namespace vex;
competition Competition;
brain Brain;

// Hardware
controller Controller1 = controller(primary);

motor MotorFL = motor(PORT20, ratio18_1, false);
motor MotorRL = motor(PORT14, ratio18_1, false);
motor MotorFR = motor(PORT1, ratio18_1, true);
motor MotorRR = motor(PORT16, ratio18_1, true);

inertial Motion = inertial(PORT3);
motor_group LeftDriveSmart = motor_group(MotorFL, MotorRL);
motor_group RightDriveSmart = motor_group(MotorFR, MotorRR);
smartdrive Drivetrain = smartdrive(LeftDriveSmart, RightDriveSmart, Motion, 319.19, 320, 40, mm, 1);

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

// Settings
const int MOTOR_TORQUE = 80;
const double speedMultiplier = .3;
const double spinMultiplier = .3;

// Global Variables
double leftSpeed = 0;
double rightSpeed = 0;

// Constants
const double TILE_LENGTH = 8;
const int TURN_ERROR = -6;
const int SCREENX = 480;
const int SCREENY = 272;

// thread callbacks
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
    motorTempAvg = (MotorFR.temperature(percent) + MotorFL.temperature(percent)
                    + MotorRR.temperature(percent) + MotorRL.temperature(percent)) / 4; // Adds up & divides all temperatures for average
    
    Brain.Screen.setFont(mono40);
    Brain.Screen.setPenColor(white);

    // Printing all motor temperatures & average
    Brain.Screen.setCursor(1, 1);
    Brain.Screen.print("%d", static_cast<int>(MotorFR.temperature(percent)));
    Brain.Screen.setCursor(7, 1);
    Brain.Screen.print("%d", static_cast<int>(MotorFL.temperature(percent)));
    Brain.Screen.setCursor(1, 23);
    Brain.Screen.print("%d", static_cast<int>(MotorRR.temperature(percent)));
    Brain.Screen.setCursor(7, 23);
    Brain.Screen.print("%d", static_cast<int>(MotorRL.temperature(percent)));
    Brain.Screen.setCursor(3, 7);
    Brain.Screen.print("MotorAVG: %d", static_cast<int>(motorTempAvg));
  }
}