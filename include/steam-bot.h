#include "vex.h"

#include <chrono>
#include <iostream>
#include <unordered_map>

enum DriveType { four_wheel, two_wheel, x_drive };

class Util {
public:
  static void measureTimeOut(int start);
  static int getTime();
};

class SteamBotMotor {
private:
  bool emptyConstructor;

  bool toggleable;
  bool reversed;
  int velocity;
  bool active;

  bool reversePolarity;
  gearSetting gear;
  int32_t port;

  vex::motor vexMotor = vex::motor(port, gear, reversePolarity);

  void setActive(bool newActive) { this->active = newActive; }

public:
  bool checkIfEmpty();

  SteamBotMotor(int32_t port, vex::gearSetting gear, bool reversePolarity,
                bool toggleable);
  SteamBotMotor();

  void spin(vex::directionType dir, int velocity);
  void spin();

  void spinForDegrees(int distance, bool wait);
  void spinForTurns(int distance, bool wait);

  void setVelocity(int newVelocity);
  int getVelocity();

  bool isReversed();
  void reverse();

  bool isActive();

  bool isToggleable();
  void toggle();

  void stop();
};

class WheelConfiguration {
private:
  bool emptyConstructor;

  std::unordered_map<std::string, SteamBotMotor> motors;
  DriveType driveType;

public:
  WheelConfiguration(std::unordered_map<std::string, SteamBotMotor> motors,
                     DriveType driveType);
  WheelConfiguration();

  std::unordered_map<std::string, SteamBotMotor> getMotors();
  DriveType getDriveType();
};

class Drivetrain {
private:
  WheelConfiguration config;
  DriveType type;

  int degreesPerAngle;

  bool emptyConstructor;

public:
  Drivetrain(WheelConfiguration config, int degreesPerAngle);
  Drivetrain();

  void move(int degrees, int velocity, bool wait);
  void turn(int angle, int velocity, bool wait);

  void moveFor(int mm, int velocity, bool wait);
};

class Configuration {
private:
  std::unordered_map<std::string, SteamBotMotor> motors;
  WheelConfiguration config;
  Drivetrain drivetrain;

public:
  Configuration(std::unordered_map<std::string, SteamBotMotor> motors,
                DriveType type, int degreesPerAngle);

  std::unordered_map<std::string, SteamBotMotor> getMotors();
  SteamBotMotor getMotor(std::string name);

  Drivetrain getDrivetrain();
};

class CompetitionManager {
private:
  vex::competition comp;

public:
  CompetitionManager(void (*driver)(), void (*auton)());
  vex::competition getCurrentCompetition();
};
