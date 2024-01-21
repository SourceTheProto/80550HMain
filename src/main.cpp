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
#define D_BARRIER_UP ButtonUp
#define D_BARRIER_DOWN ButtonDown

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

motor BarrierR = motor(PORT4, ratio18_1, true);
motor BarrierL = motor(PORT19, ratio18_1, false);

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
  #define DRIVE_TORQUE 80
  #define INTAKE_TORQUE 100
  #define INTAKE_SPEED 50
  #define BARRIER_TORQUE 100
  #define BARRIER_SPEED 50
  #define DEBUG_TIMEOUT_SECONDS 10

// Enumerations
enum directional {REVERSE = -1, FORWARD = 1};
enum positional {UP = 1, DOWN = 0};
enum turnMethod {FOR, TO};
enum menuLink {NONE, COMPETITION, DRIVER_CONTROL, AUTONOMOUS, ENABLED, DISABLED};

// Constants
#define TILE_LENGTH 24
#define WHEEL_DIAMETER 4
const float pi = 3.141592;

struct menuButton {
  char* text;
  menuLink menu = NONE;
};

class MainMenu {
public:

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
          if (buttonIndex < 0) {buttonIndex = (menuButtons.size()-1);}
          while (Controller1.ButtonLeft.pressing()) {wait(5, msec);}
          break;
        } else if (Controller1.ButtonRight.pressing()) {
          buttonIndex++;
          if (buttonIndex > menuButtons.size()-1) {buttonIndex = 0;}
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

class AutonSelector {
public:

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
          if (buttonIndex < 0) {buttonIndex = (menuButtons.size()-1);}
          while (Controller1.ButtonLeft.pressing()) {wait(5, msec);}
          break;
        } else if (Controller1.ButtonRight.pressing()) {
          buttonIndex++;
          if (buttonIndex > menuButtons.size()-1) {buttonIndex = 0;}
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

  AutonSelector() {
    // Competition Button
    EnabledButton.text = "  Auton Enabled  ";
    EnabledButton.menu = ENABLED;

    // Driver Control Button
    DisabledButton.text = "  Auton Disabled ";
    DisabledButton.menu = DISABLED;

    menuButtons.reserve(2);
    menuButtons.push_back(EnabledButton);
    menuButtons.push_back(DisabledButton);
  }

private:

  std::vector<menuButton> menuButtons;

  // Buttons
  menuButton EnabledButton;
  menuButton DisabledButton;

  void clearDisplay() {
    Controller1.Screen.setCursor(4, 1);
    Controller1.Screen.print("                            ");
  }
};

// Global Variables
double leftSpeed = 0;
double rightSpeed = 0;

double barrierLBasePos;
double barrierRBasePos;
double barrierLUpPos;
double barrierRUpPos;
bool barrierState = DOWN;
bool barrierLastState;

bool autonEnabled = true;

std::vector<char*> debugBuffer;
double debugTimer = 0;

// Thread callbacks
void DisplayManager();
void barrierControlThread();

// Function declarations
void autonomous();
void driverControl();
void drive(directional direction, double dist);
void turn(turnMethod method, int angle);
void calibrate();
void printDebug(char* message);

int main() {
  // Setting Motor Speeds
  Intake.setVelocity(INTAKE_SPEED, percent);
  Intake.setMaxTorque(INTAKE_TORQUE, percent);

  // Setting Barrier settings and variables
  BarrierR.setVelocity(BARRIER_SPEED, percent);
  BarrierR.setMaxTorque(BARRIER_TORQUE, percent);
  BarrierL.setVelocity(BARRIER_SPEED, percent);
  BarrierL.setMaxTorque(BARRIER_TORQUE, percent);
  BarrierL.stop(hold);
  BarrierR.stop(hold);

  barrierLBasePos = BarrierL.position(deg);
  barrierRBasePos = BarrierR.position(deg);
  barrierLUpPos = barrierLBasePos + 88;
  barrierRUpPos = barrierRBasePos + 88;

  thread Calibrator = thread(calibrate);

  MainMenu StartMenu;
  menuLink selectedMenu = StartMenu.run();

  if (selectedMenu == COMPETITION) {
    Competition.autonomous(autonomous);
    Competition.drivercontrol(driverControl);

    AutonSelector selector;
    menuLink selectedAuton = selector.run();
    if (selectedAuton == ENABLED) autonEnabled = true;
    if (selectedAuton == DISABLED) autonEnabled = false;

	  Calibrator.join();
    thread BarrierControl = thread(barrierControlThread);
    thread DM = thread(DisplayManager);
  } else if (selectedMenu == DRIVER_CONTROL) {
    thread BarrierControl = thread(barrierControlThread);
    Calibrator.join();
    thread DM = thread(DisplayManager);
    driverControl();
  } else if (selectedMenu == AUTONOMOUS) {
	  Calibrator.join();
    thread DM = thread(DisplayManager);
    autonomous();
  }

  while (true) {
    wait(100, msec);
  }
}


/*$$$$$\              $$\                                                                               
$$  __$$\             $$ |                                                                              
$$ /  $$ |$$\   $$\ $$$$$$\    $$$$$$\  $$$$$$$\   $$$$$$\  $$$$$$\$$$$\   $$$$$$\  $$\   $$\  $$$$$$$\ 
$$$$$$$$ |$$ |  $$ |\_$$  _|  $$  __$$\ $$  __$$\ $$  __$$\ $$  _$$  _$$\ $$  __$$\ $$ |  $$ |$$  _____|
$$  __$$ |$$ |  $$ |  $$ |    $$ /  $$ |$$ |  $$ |$$ /  $$ |$$ / $$ / $$ |$$ /  $$ |$$ |  $$ |\$$$$$$\  
$$ |  $$ |$$ |  $$ |  $$ |$$\ $$ |  $$ |$$ |  $$ |$$ |  $$ |$$ | $$ | $$ |$$ |  $$ |$$ |  $$ | \____$$\ 
$$ |  $$ |\$$$$$$  |  \$$$$  |\$$$$$$  |$$ |  $$ |\$$$$$$  |$$ | $$ | $$ |\$$$$$$  |\$$$$$$  |$$$$$$$  |
\__|  \__| \______/    \____/  \______/ \__|  \__| \______/ \__| \__| \__| \______/  \______/ \______*/

void autonomous() {
  if (!autonEnabled) return;

  Intake.spin(forward);
  drive(FORWARD, 2);
  turn(FOR, 90);
  Intake.spin(reverse);
  drive(FORWARD, .25);
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
      while (desiredHeading > 360) desiredHeading -= 360;
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
    double logDifference = log((desiredHeading - (Motion.heading())));
    leftSide((TURN_VELOCITY/10) * logDifference, percent);
    rightSide((-1*(TURN_VELOCITY/10) * logDifference), percent);
  }
  stopDriveMotors(hold);
  wait(.2, sec);
}

void barrierControlThread() {
  while (true) {
    if (barrierLastState == barrierState) {
      wait(5, msec);
      continue;
    }
    switch (barrierState) {
      case UP:
        barrierLastState = barrierState;
        BarrierL.spin(forward);
        BarrierR.spin(forward);
        while ((BarrierL.position(deg) < barrierLUpPos || BarrierR.position(deg) < barrierRUpPos) && !Controller1.D_BARRIER_DOWN.pressing()) {
          wait(5, msec);
        }
        BarrierL.stop(hold);
        BarrierR.stop(hold);
        break;
      case DOWN:
        barrierLastState = barrierState;
        BarrierL.spin(reverse);
        BarrierR.spin(reverse);
        while ((BarrierL.position(deg) > barrierLBasePos || BarrierR.position(deg) > barrierRBasePos) && !Controller1.D_BARRIER_UP.pressing()) {
          wait(5, msec);
        }
        BarrierL.stop(hold);
        BarrierR.stop(hold);
        break;
    }

    wait(5, msec);
  }
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

    // Barrier Control
    if (Controller1.D_BARRIER_UP.pressing() && !Controller1.D_BARRIER_DOWN.pressing()) barrierState = UP;
    if (Controller1.D_BARRIER_DOWN.pressing() && !Controller1.D_BARRIER_UP.pressing()) barrierState = DOWN;
  }
}

void printDebug(char* message) {
  debugBuffer.insert(debugBuffer.begin(), message);
  debugTimer = DEBUG_TIMEOUT_SECONDS;
}

void DisplayManager() {
  double motorTempAvg = 0;
  
  while (true) {
    Brain.Screen.clearScreen();
    while (debugTimer == 0) {
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
      wait(100, msec);
    }

    Brain.Screen.clearScreen(black);
    Brain.Screen.setFont(mono20);
    Brain.Screen.setPenColor(white);

    while (debugBuffer.size() > 15) debugBuffer.pop_back();
    
    for (int i = 1; i <= debugBuffer.size(); i++) {
      Brain.Screen.setCursor(i, 1);
      Brain.Screen.print(debugBuffer[i-1]);
    }
	debugTimer -= 0.2;
	while (debugTimer != 0 && debugTimer != DEBUG_TIMEOUT_SECONDS) {
		debugTimer -= 0.2;
    if (debugTimer < 0) debugTimer = 0;
    wait(200, msec);
	}
  }
}

void calibrate() {
  Motion.calibrate();
  Brain.Screen.clearScreen(orange);
  Brain.Screen.setFont(mono40);
  Brain.Screen.setPenColor(white);
  Brain.Screen.setFillColor(orange);
  Brain.Screen.print("Calibrating...");
  while (Motion.isCalibrating()) {wait(5, msec);}
  Brain.Screen.clearScreen(black);
}