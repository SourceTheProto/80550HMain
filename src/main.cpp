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

/*
$$$$$$$\  $$\                 $$\ $$\                               
$$  __$$\ \__|                $$ |\__|                              
$$ |  $$ |$$\ $$$$$$$\   $$$$$$$ |$$\ $$$$$$$\   $$$$$$\   $$$$$$$\ 
$$$$$$$\ |$$ |$$  __$$\ $$  __$$ |$$ |$$  __$$\ $$  __$$\ $$  _____|
$$  __$$\ $$ |$$ |  $$ |$$ /  $$ |$$ |$$ |  $$ |$$ /  $$ |\$$$$$$\  
$$ |  $$ |$$ |$$ |  $$ |$$ |  $$ |$$ |$$ |  $$ |$$ |  $$ | \____$$\ 
$$$$$$$  |$$ |$$ |  $$ |\$$$$$$$ |$$ |$$ |  $$ |\$$$$$$$ |$$$$$$$  |
\_______/ \__|\__|  \__| \_______|\__|\__|  \__| \____$$ |\_______/ 
                                                $$\   $$ |          
                                                \$$$$$$  |          
                                                 \_____*/
#define A_FORWARD Axis3
#define A_TURN Axis1
#define D_WINGS_ENABLE ButtonR1
#define D_WINGS_DISABLE ButtonL1
#define D_INTAKE_FORWARD ButtonR2
#define D_INTAKE_REVERSE ButtonL2

#include "vex.h"
#include <vector>
using namespace vex;
competition Competition;

/*
$$\   $$\                           $$\                                             
$$ |  $$ |                          $$ |                                            
$$ |  $$ | $$$$$$\   $$$$$$\   $$$$$$$ |$$\  $$\  $$\  $$$$$$\   $$$$$$\   $$$$$$\  
$$$$$$$$ | \____$$\ $$  __$$\ $$  __$$ |$$ | $$ | $$ | \____$$\ $$  __$$\ $$  __$$\ 
$$  __$$ | $$$$$$$ |$$ |  \__|$$ /  $$ |$$ | $$ | $$ | $$$$$$$ |$$ |  \__|$$$$$$$$ |
$$ |  $$ |$$  __$$ |$$ |      $$ |  $$ |$$ | $$ | $$ |$$  __$$ |$$ |      $$   ____|
$$ |  $$ |\$$$$$$$ |$$ |      \$$$$$$$ |\$$$$$\$$$$  |\$$$$$$$ |$$ |      \$$$$$$$\ 
\__|  \__| \_______|\__|       \_______| \_____\____/  \_______|\__|       \_______|*/

brain Brain;
controller Controller1 = controller(primary);
inertial Motion = inertial(PORT3);

motor MotorFL = motor(PORT14, ratio18_1, true);
motor MotorRL = motor(PORT16, ratio18_1, true);
motor MotorFR = motor(PORT20, ratio18_1, false);
motor MotorRR = motor(PORT1, ratio18_1, false);

motor Intake = motor(PORT2, ratio18_1, false);

digital_out Wings = digital_out(Brain.ThreeWirePort.A);

/*
 $$$$$$\             $$\     $$\     $$\                               
$$  __$$\            $$ |    $$ |    \__|                              
$$ /  \__| $$$$$$\ $$$$$$\ $$$$$$\   $$\ $$$$$$$\   $$$$$$\   $$$$$$$\ 
\$$$$$$\  $$  __$$\\_$$  _|\_$$  _|  $$ |$$  __$$\ $$  __$$\ $$  _____|
 \____$$\ $$$$$$$$ | $$ |    $$ |    $$ |$$ |  $$ |$$ /  $$ |\$$$$$$\  
$$\   $$ |$$   ____| $$ |$$\ $$ |$$\ $$ |$$ |  $$ |$$ |  $$ | \____$$\ 
\$$$$$$  |\$$$$$$$\  \$$$$  |\$$$$  |$$ |$$ |  $$ |\$$$$$$$ |$$$$$$$  |
 \______/  \_______|  \____/  \____/ \__|\__|  \__| \____$$ |\_______/ 
                                                   $$\   $$ |          
                                                   \$$$$$$  |          
                                                    \_____*/
  // Auton
  const int DRIVE_VELOCITY = 20;
  const int TURN_VELOCITY = 25;

  // Driver Control
  const double DCspeedMult = .6;
  const double DCspinMult = .3;
  const int DRIVE_TORQUE = 80;
  const int INTAKE_TORQUE = 100;
  const int INTAKE_SPEED = 50;

// Enumerations
enum directional {REVERSE = -1, FORWARD = 1};
enum turnMethod {FOR, TO};
enum menuLink {NONE, COMPETITION, DRIVER_CONTROL, AUTONOMOUS};

// Constants
const int TILE_LENGTH = 24;
const int WHEEL_DIAMETER = 4;
const int SCREENX = 480;
const int SCREENY = 272;
const int C_SCREENX = 20;
const float pi = 3.141592;

class MainMenu {
public:

  struct menuButton {
    char* text;
    menuLink menu = NONE;
  };

  menuLink run() {
    int buttonIndex = 0;
    
    while (true)
    {
      clearDisplay();
      Controller1.Screen.setCursor(4, 1);
      Controller1.Screen.print("< ");
      Controller1.Screen.print(menuButtons[buttonIndex].text);
      Controller1.Screen.print(" >");
      
      while (true)
      {
        if (Controller1.ButtonLeft.pressing()) {
          buttonIndex--;
          if (buttonIndex < 0) {buttonIndex = 2;}
          while (Controller1.ButtonLeft.pressing()) {wait(5, msec);}
          break;
        } else if (Controller1.ButtonRight.pressing()) {
          buttonIndex++;
          if (buttonIndex > 2) {buttonIndex = 0;}
          while (Controller1.ButtonRight.pressing()) {wait(5, msec);}
          break;
        } else if (Controller1.ButtonA.pressing()) {
          while (Controller1.ButtonA.pressing()) {wait(5, msec);}
          clearDisplay();
          return menuButtons[buttonIndex].menu;
        }
      }
    }
  }

  MainMenu() {
    // Competition Button
    CompetitionButton.text = "Competition Mode";
    CompetitionButton.menu = COMPETITION;

    // Driver Control Button
    DCButton.text = "   Driver Control  ";
    DCButton.menu = DRIVER_CONTROL;

    // Autonomous Button
    AutonButton.text = "   Autonomous   ";
    AutonButton.menu = AUTONOMOUS;

    menuButtons.reserve(3);
    menuButtons.push_back(CompetitionButton);
    menuButtons.push_back(DCButton);
    menuButtons.push_back(AutonButton);
  }

private:

  std::vector<menuButton> menuButtons;

  // Buttons
  menuButton CompetitionButton;
  menuButton DCButton;
  menuButton AutonButton;

  void clearDisplay() {
    Controller1.Screen.setCursor(4, 1);
    Controller1.Screen.print("                            ");
  }
};


// Global Variables
double leftSpeed = 0;
double rightSpeed = 0;

// Thread callbacks
void temperatureMonitor();

// Function declarations
void autonomous();
void driverControl();
void drive(directional direction, double dist);
void turn(turnMethod method, int angle);

int main() {
  Motion.calibrate();
  Brain.Screen.clearScreen(red);
  Brain.Screen.setFont(mono40);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(red);
  Brain.Screen.print("Calibrating...");

  MainMenu StartMenu;
  
  menuLink selectedMenu = StartMenu.run();

  if (selectedMenu == COMPETITION) {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(driverControl);
    while (Motion.isCalibrating()) {wait(5, msec);}
    thread TMON = thread(temperatureMonitor);
  } else if (selectedMenu == DRIVER_CONTROL) {
    while (Motion.isCalibrating()) {wait(5, msec);}
    thread TMON = thread(temperatureMonitor);
    driverControl();
  } else if (selectedMenu == AUTONOMOUS) {
      while (Motion.isCalibrating()) {wait(5, msec);}
      thread TMON = thread(temperatureMonitor);
      autonomous();
  }

  while (true) {
    wait(100, msec);
  }
}

/*
 $$$$$$\              $$\                                                                               
$$  __$$\             $$ |                                                                              
$$ /  $$ |$$\   $$\ $$$$$$\    $$$$$$\  $$$$$$$\   $$$$$$\  $$$$$$\$$$$\   $$$$$$\  $$\   $$\  $$$$$$$\ 
$$$$$$$$ |$$ |  $$ |\_$$  _|  $$  __$$\ $$  __$$\ $$  __$$\ $$  _$$  _$$\ $$  __$$\ $$ |  $$ |$$  _____|
$$  __$$ |$$ |  $$ |  $$ |    $$ /  $$ |$$ |  $$ |$$ /  $$ |$$ / $$ / $$ |$$ /  $$ |$$ |  $$ |\$$$$$$\  
$$ |  $$ |$$ |  $$ |  $$ |$$\ $$ |  $$ |$$ |  $$ |$$ |  $$ |$$ | $$ | $$ |$$ |  $$ |$$ |  $$ | \____$$\ 
$$ |  $$ |\$$$$$$  |  \$$$$  |\$$$$$$  |$$ |  $$ |\$$$$$$  |$$ | $$ | $$ |\$$$$$$  |\$$$$$$  |$$$$$$$  |
\__|  \__| \______/    \____/  \______/ \__|  \__| \______/ \__| \__| \__| \______/  \______/ \______*/

void autonomous() {
  drive(FORWARD, 2);
  Wings.set(true);
  turn(FOR, 90);
  drive(FORWARD, .25);
  Wings.set(false);
}

void drive(directional direction, double dist)
{
  double initialPosition = MotorFL.position(rev);
  double initialHeading = Motion.heading();

  dist = (dist*(TILE_LENGTH/2))/(pi*WHEEL_DIAMETER);

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
  
  while ((double)MotorFL.position(rev) <= initialPosition + dist)
  {
    double angleDist = (Motion.heading() - initialHeading);
    if(angleDist > 180) {
      angleDist -= 360;
    }
    leftSide(DRIVE_VELOCITY - angleDist, percent);
    rightSide(DRIVE_VELOCITY + angleDist, percent);
  }
  stopDriveMotors(hold);
  turn(TO, initialHeading);
  wait(.2, sec);
}

void turn(turnMethod method, int angle) {
  double desiredHeading;
  switch (method) {
    case FOR:
      desiredHeading = Motion.heading() + angle;
      while (desiredHeading > 360) {
        desiredHeading -= 360;
      }
      break;
    
    case TO:
      desiredHeading = angle;
      break;
  }

  leftSide(TURN_VELOCITY, percent);
  rightSide(-1*TURN_VELOCITY, percent);
  spinDriveMotors(forward);

  while ((double)Motion.heading() <= desiredHeading)
  {
    double logDifference = log((desiredHeading - (Motion.heading()))+1);
    leftSide((TURN_VELOCITY/10) * logDifference, percent);
    rightSide(-1*(TURN_VELOCITY/10) * logDifference, percent);
  }
  stopDriveMotors(hold);
  wait(.2, sec);
}

/*
$$$$$$$\            $$\                                
$$  __$$\           \__|                               
$$ |  $$ | $$$$$$\  $$\ $$\    $$\  $$$$$$\   $$$$$$\  
$$ |  $$ |$$  __$$\ $$ |\$$\  $$  |$$  __$$\ $$  __$$\ 
$$ |  $$ |$$ |  \__|$$ | \$$\$$  / $$$$$$$$ |$$ |  \__|
$$ |  $$ |$$ |      $$ |  \$$$  /  $$   ____|$$ |      
$$$$$$$  |$$ |      $$ |   \$  /   \$$$$$$$\ $$ |      
\_______/ \__|      \__|    \_/     \_______|\__|
 $$$$$$\                       $$\                         $$\ 
$$  __$$\                      $$ |                        $$ |
$$ /  \__| $$$$$$\  $$$$$$$\ $$$$$$\    $$$$$$\   $$$$$$\  $$ |
$$ |      $$  __$$\ $$  __$$\\_$$  _|  $$  __$$\ $$  __$$\ $$ |
$$ |      $$ /  $$ |$$ |  $$ | $$ |    $$ |  \__|$$ /  $$ |$$ |
$$ |  $$\ $$ |  $$ |$$ |  $$ | $$ |$$\ $$ |      $$ |  $$ |$$ |
\$$$$$$  |\$$$$$$  |$$ |  $$ | \$$$$  |$$ |      \$$$$$$  |$$ |
 \______/  \______/ \__|  \__|  \____/ \__|       \______/ \__| */

void driverControl() {
  // Setting Drive Motor Settings
  MotorFR.setMaxTorque(DRIVE_TORQUE, percent);
  MotorRR.setMaxTorque(DRIVE_TORQUE, percent);
  MotorFL.setMaxTorque(DRIVE_TORQUE, percent);
  MotorRL.setMaxTorque(DRIVE_TORQUE, percent);

  // Setting Intake Motor Settings
  Intake.setMaxTorque(INTAKE_TORQUE, percent);
  Intake.setVelocity(INTAKE_SPEED, percent);

  while (true) {
    // Drive Control
    // Setting variables
    leftSpeed = (Controller1.A_FORWARD.position()*DCspeedMult) + (Controller1.A_TURN.position()*DCspinMult);
    rightSpeed = (Controller1.A_FORWARD.position()*DCspeedMult) - (Controller1.A_TURN.position()*DCspinMult);
    // Applying Movement
    if (fabs(leftSpeed)+fabs(rightSpeed) > 5) {
      leftSide(leftSpeed, percent);
      rightSide(rightSpeed, percent);
      spinDriveMotors(forward);
    } else {
      stopDriveMotors(brake);
    }

    // Wing Control
    if (Controller1.D_WINGS_ENABLE.pressing()) {
      Wings.set(true);
    } else if (Controller1.D_WINGS_DISABLE.pressing()) {
      Wings.set(false);
    }

    if (Controller1.D_INTAKE_FORWARD.pressing() && !Controller1.D_INTAKE_REVERSE.pressing()) {
      Intake.spin(forward);
    } else if (Controller1.D_INTAKE_REVERSE.pressing() && !Controller1.D_INTAKE_FORWARD.pressing()) {
      Intake.spin(reverse);
    } else {
      Intake.stop();
    }
  }
}

void temperatureMonitor() {
  double motorTempAvg = 0;
  Brain.Screen.clearScreen();
  Brain.Screen.setFillColor(black);
  Brain.Screen.setPenColor(black);
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
  }
}