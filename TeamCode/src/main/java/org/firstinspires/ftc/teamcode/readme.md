# Automaducks Pond - Sample Code

**Welcome to the Automaducks sample project!**

## Welcome to Pond

Pond is a library built by the [FTC 23206 Automaducks](https://www.automaducks.com/) team which aims allow teams to develop highly modular and performant robot code while maintaining a very simple and compact programming model. It empowers teams to quickly starting developing advanced robot code while allowing for growth and extensibility as teams progresses.

Key Features:

* Use of Real-Time periodic execution model which avoids robot being unresponsive
* Provides common functionality to be leveraged and extended by FTC Teams: Subsystems, Commands, Robot Control, etc.
* Does not take any dependency on FTC SDK, thus not requiring library upgrades when new FTC SDK Versions are published

> See [Pond Documentation](https://...) for more information.
  
## Suggest Project Structure

| Directory    | Purpose                                                                                         |
|--------------|-------------------------------------------------------------------------------------------------|
| `operations` | Home of all the FTC Robot operations such as DriveOp, Autonomous, Robot Tunning, etc.           |
| `subsystems` | Modules that control your individual robot systems such as linenar slides, intakes, claws, etc. |
| `utility`    | Helper classes for your project                                                                 |

## Integrations with other community libraries

Although Pond doesn't require you to use any other community provided libraries, it is fully expected that FTC Teams use the beast from the community. Based on this, this startup repo already integrates with the following:

* [FTC Dashboard](https://acmerobotics.github.io/ftc-dashboard/): great tool for changing robot configurations without the need to redeploy code and also for graphing robot operational parameters
* [Road Runner](https://github.com/acmerobotics/road-runner) Trajectory planning for FTC robots
* [MeepMeep](https://github.com/rh-robotics/MeepMeep) Path visualization for RoadRunner ([Learn Road Runner - Meep Meep](https://learnroadrunner.com/tool/meepmeep.html))

## Team TODOs

* [ ] Kinematics
  * [x] Rename VectorOrientation to Pose
  * [x] Split mecanum kinematics
  * [x] Split dead wheels kinematics
  * [ ] Return velocity from Mecanum Kinematics
* [ ] Mecanum Chassis
  * [ ] Implement basic Mecanum Chassis
* [ ] Localizer
  * [x] Return a compound localization object containing position and speeds
  * [x] Remove GetLocalizer from IDriveTrain?
* [ ] Subsystems
  * [x] Allow find by interface
  * [ ] Allow find by name?
  * [ ] Recursive find
* [ ] Commands
  * [x] Implement Triggers for Event Driver programming
  * [ ] Port old FSM features to new command system
  * [ ] Support for any/sequenctial/parallel
  * [ ] Support for runOnce commands
  * [ ] Support for strongly typed commands
* [ ] Autonomous
* [ ] Configuration
* [ ] FTC Dashboard
  * [x] Sample configuration
  * [ ] Robot field localization
* [ ] Docs
  * [ ] Convert JavaDocs to Markdown
  * [ ] Publish Pond documentation and new library version
