# Unified Program for team 80550H
This program is the unified testing and competition mode program for the 80550H team, made to make development and testing simpler, without having to manage multiple codebases simulaneously and keeping seperate versions.

## Mode selector
The program comes with a mode selector UI that shows on the controller when the program is started. It allows the user to select between:
* Driver Control
* Autonomous
* Competition

The first two modes are to be run for testing each individual component without needing a competition switch. The last one sets up the callbacks for the competition itself. Each selector utilizes the same functions in code, so a change to the driver control code, for example, would reflect both when you select driver control and during the driver control period in a competition.

### Auton Selector
When Autonomous or Competition is selected, the menu will prompt again, asking which side of autonomous to execute (the far side of the goal was never implemented), or to disable it entirely.

### Technical Details
This menu system was done all through a MainMenu class that handled the creation of menus and the user prompts, along with assisting classes to make the return value handling easier to use.

## Display Manager
This program comes with a display manager run on a separate thread during all runs of the program past the main menu. The display manager can display any text by default (normally motor temps), but then switch over to error and debug messages when called. When the `printDebug()` function is called with text, that text is added to a vector to be displayed on the screen. When the vector of messages contains at least one item, the default screen is replaced with the debug messages for the time defined by `DEBUG_TIMEOUT_SECONDS`, then returning to the original screen, and the debug buffer is cleared.

## Bindings System
There is also a simple bindings system implemented to be able to change binds more simply when the driver asks, utilizing the preprocessor. Binds are simply defined using the #define directive, and the buttons are inserted into the code, making changing binds trivial. No more is searching through the code for every instance of a certain bind.

## Autonomous Functions
There are two autonomous functions included with automatic heading correction: `drive` and `turn`.

### Drive Function
The drive function will set the motors to drive forward, but also continuously checks the bot's heading to check for correction. As the heading deviates from the initial heading, the bot will add velocity to the left or right side to straighten out the bot during driving, making it even resilient to direct pushing. Although, this can sometimes get unstable with time driven in some cases.

### Turn Function
The turn function is fairly straight forward: it turns until the final desired heading is achieved, while slowing the bot down when it comes near the desired angle. It has two different modes: `FOR` and `TO`. `FOR` turns until it has turned to its initial heading + the degree angle specified, and `TO` turns until the specified heading is achieved.