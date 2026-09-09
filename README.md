# 2026-REV-ION-FRC-Starter-Bot

This project provides starting Java code for the 2026 REV ION FRC Starter Bot.

For the most up-to-date information about the 2026 REV ION FRC Starter Bot, watch the [REV website](https://docs.revrobotics.com/frc-kickoff-concepts/).

## Deploying this project to your robot

Before you can deploy this project to your robot, you'll need to set your team number in `/.wpilib/wpilib_preferences.json` or via the "Set Team Number" tool in WPILib VSCode. You will need to update WPILib VSCode to a 2026+ release to build this project.

## Driving the 2026 REV ION FRC Starter Bot

All controls for the Starter Bot are located in `/src/main/java/frc/robot/RobotContainer.java` and can be configured with WPILib's Command Controller API. The project is configured for an Xbox controller by default, so other controllers may require some additional setup or changes.

The robot's starting/zero configuration is:

- intake in

### Default Controls

| Button | Function |
| --- | --- |
| Left stick | Drive/strafe drivetrain |
| Right stick | Rotate drivetrain |
| Start button | Zero swerve heading |
| Left stick button | Set swerve wheels to an X |
| Y button | Run shooter and feeder |
| Right trigger | Run fuel intake |
| Left trigger | Reverse fuel intake |

## Other features

### CAN IDs

The CAN IDs for the Starter Bot motor controller are defined in `/src/main/java/frc/robot/Constants.java`.

The default IDs are:

| Subsystem | Function | Type | CAN ID |
| --- | --- | --- | --- |
| Intake | Intake Motor | SPARK Flex | 2 |
| | Conveyor Motor | SPARK Flex | 4 |
| Shooter | Feeder Motor | SPARK Flex | 5 |
| | Flywheel Motor (Leader on the Right) | SPARK Flex | 6 |
| | Flywheel Motor (Follower on the Left) | SPARK Flex | 7 |
| Drive | Front Left Driving Motor | SPARK MAX | 15 |
| | Front Right Driving Motor | SPARK MAX | 11 |
| | Rear Right Driving Motor | SPARK MAX | 9 |
| | Rear Left Driving Motor | SPARK MAX | 13 |
| | Front Left Turning Motor | SPARK MAX | 14 |
| | Front Right Turning Motor | SPARK MAX | 10 |
| | Rear Right Turning Motor | SPARK MAX | 8 |
| | Rear Left Turning Motor | SPARK MAX | 12 |

### Configuration

All configuration objects are instantiated in `/src/main/java/frc/robot/Configs.java` and called by each subsystem

### MAXMotion

The Shooter Flywheel implements MAXMotion Velocity Mode

### Simulation

Simulation coming soon...

Check out the [REVLib simulation docs](https://docs.revrobotics.com/revlib/spark/sim) for more information on setting up and running a simulation. Select `NetworkTables` > `SmartDashboard` to open the `Mechanism2d` windows in the Simulation GUI. These displays will also update with a robot connected, so play around with them and add them to your dashboard! They can be configured in `/src/main/java/frc/robot/Constants.java`.

### Debugging

Individual mechanisms can also be controlled via SmartDashboard:

- File -> Open -> 2026-Starter-Bot-SmartDashboard.xml (in the root of this repo)
- The Upper left corner shows the currently running subsystems
- The Intake and Shooter subsystems contain Commands and status
  - In a subsystem, only a single command may run at a time
  - Pressing the `Start` button starts the Command and is replaced by a `Cancel` button, which stops the command
 
#### `IntakeSubsystem`
- **`Intake`** : Runs the Intake and Conveyor into the robot
- **`Extake`** : Runs the Intake and Conveyor out of the robot

#### `ShooterSubsystem`
- **`Flywheel`** : spins the flywheel up to its target velocity
- **`Feeder`** : feeds the balls to the flywheel and keeps the flywheel spinning
