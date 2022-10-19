#include "steam-bot.h"

/**
  Util
    Utility functions used everywhere.
**/

const auto p1 = std::chrono::system_clock::now();

int Util::getTime() {
  return std::chrono::duration_cast<std::chrono::seconds>(p1.time_since_epoch())
      .count();
}

void Util::measureTimeOut(int start) {
  std::cout << " [" << (getTime() - start) << "ms]" << std::endl;
}

/**
  SteamBotMotor
    Wrapper for vex::motor with increased utility.
**/

bool SteamBotMotor::checkIfEmpty() {
  if (this->emptyConstructor) {
    std::cout << "Default constructor of SteamBotMotor not supported."
              << std::endl;

    return true;
  }

  return false;
}

SteamBotMotor::SteamBotMotor(int32_t port, vex::gearSetting gear,
                             bool reversePolarity, bool toggleable = false) {
  this->reversePolarity = reversePolarity;
  this->port = port;
  this->gear = gear;

  this->toggleable = toggleable;

  this->emptyConstructor = false;
}

SteamBotMotor::SteamBotMotor() { this->emptyConstructor = true; }

void SteamBotMotor::spin(vex::directionType dir, int velocity) {
  if (checkIfEmpty())
    return;

  this->setVelocity(velocity);

  this->vexMotor.spin(dir);
}

void SteamBotMotor::spin() {
  if (checkIfEmpty())
    return;

  this->vexMotor.spin(this->reversed ? vex::reverse : vex::forward);
}

void SteamBotMotor::spinForDegrees(int distance, bool wait) {
  if (checkIfEmpty())
    return;

  this->vexMotor.spinFor(this->reversed ? -distance : distance, vex::degrees,
                         wait);
}

void SteamBotMotor::spinForTurns(int distance, bool wait) {
  if (checkIfEmpty())
    return;

  this->vexMotor.spinFor(this->reversed ? -distance : distance, vex::turns,
                         wait);
}

void SteamBotMotor::setVelocity(int newVelocity) {
  if (checkIfEmpty())
    return;

  this->velocity = newVelocity;

  this->vexMotor.setVelocity(velocity, vex::percent);
}

int SteamBotMotor::getVelocity() {
  if (checkIfEmpty())
    return -1;
  return this->velocity;
}

bool SteamBotMotor::isReversed() {
  if (checkIfEmpty())
    return false;
  return this->reversed;
}

void SteamBotMotor::reverse() {
  if (checkIfEmpty())
    return;
  this->reversed = !this->reversed;
}

bool SteamBotMotor::isActive() {
  if (checkIfEmpty())
    return false;
  return this->active;
}

void SteamBotMotor::toggle() {
  if (checkIfEmpty())
    return;

  if (!this->toggleable)
    return;

  this->active = !this->active;

  this->setVelocity(this->active ? 100 : 0);

  if (this->active)
    this->spin();
  else
    this->stop();
}

bool SteamBotMotor::isToggleable() {
  if (checkIfEmpty())
    return false;
  return this->toggleable;
}

void SteamBotMotor::stop() {
  if (checkIfEmpty())
    return;
  this->vexMotor.stop();
}

/**
  WheelConfiguration
    Drivetrain configuration used to determine which motors to run while
    moving/turning.
**/
WheelConfiguration::WheelConfiguration(
    std::unordered_map<std::string, SteamBotMotor> motors,
    DriveType driveType) {
  this->driveType = driveType;
  this->motors = motors;

  this->emptyConstructor = false;
}

WheelConfiguration::WheelConfiguration() { this->emptyConstructor = true; }

std::unordered_map<std::string, SteamBotMotor> WheelConfiguration::getMotors() {
  if (this->emptyConstructor) {
    std::cout << "Default constructor of WheelConfiguration not supported."
              << std::endl;
    return *new std::unordered_map<std::string, SteamBotMotor>(0);
  }
  return this->motors;
}

DriveType WheelConfiguration::getDriveType() {
  if (this->emptyConstructor) {
    std::cout << "Default constructor of WheelConfiguration not supported."
              << std::endl;
    return four_wheel;
  }
  return this->driveType;
}

/**
  Drivetrain
    Control movemenet of the robot's drivebase.
**/
Drivetrain::Drivetrain(WheelConfiguration config, int degreesPerAngle) {
  this->degreesPerAngle = degreesPerAngle;

  this->config = config;
  this->type = config.getDriveType();

  this->emptyConstructor = false;
}

Drivetrain::Drivetrain() { this->emptyConstructor = true; }

void Drivetrain::move(int degrees, int velocity, bool wait = true) {
  if (this->emptyConstructor) {
    std::cout << "Default constructor of Drivetrain not supported."
              << std::endl;
    return;
  }

  WheelConfiguration config = this->config;
  DriveType type = this->type;

  SteamBotMotor frontRightDrive;
  SteamBotMotor frontLeftDrive;
  SteamBotMotor backRightDrive;
  SteamBotMotor backLeftDrive;

  SteamBotMotor rightDrive;
  SteamBotMotor leftDrive;

  if (type == four_wheel || type == x_drive) {
    frontRightDrive = config.getMotors()["frontRightDrive"];
    frontLeftDrive = config.getMotors()["frontLeftDrive"];
    backRightDrive = config.getMotors()["backRightDrive"];
    backLeftDrive = config.getMotors()["backLeftDrive"];
  } else if (type == two_wheel) {
    rightDrive = config.getMotors()["rightDrive"];
    leftDrive = config.getMotors()["leftDrive"];
  } else {
    std::cout << "Failed to initialize drivetrain: Invalid DriveType: " << type
              << std::endl;
  }

  switch (type) {
  case four_wheel | x_drive:
    frontRightDrive.setVelocity(velocity);
    frontLeftDrive.setVelocity(velocity);
    backRightDrive.setVelocity(velocity);
    backLeftDrive.setVelocity(velocity);

    frontRightDrive.spinForDegrees(degrees, false);
    frontLeftDrive.spinForDegrees(degrees, false);
    backRightDrive.spinForDegrees(degrees, false);
    backLeftDrive.spinForDegrees(degrees, wait);

    break;
  case two_wheel:
    rightDrive.setVelocity(velocity);
    leftDrive.setVelocity(velocity);

    rightDrive.spinForDegrees(degrees, false);
    leftDrive.spinForDegrees(degrees, wait);

    break;
  default:
    std::cout << "Failed to initialize drivetrain: Invalid DriveType: " << type
              << std::endl;
  }
}

void Drivetrain::turn(int angle, int velocity, bool wait) {
  if (this->emptyConstructor) {
    std::cout << "Default constructor of Drivetrain not supported."
              << std::endl;
    return;
  }

  WheelConfiguration config = this->config;
  DriveType type = this->type;

  SteamBotMotor frontRightDrive;
  SteamBotMotor frontLeftDrive;
  SteamBotMotor backRightDrive;
  SteamBotMotor backLeftDrive;

  SteamBotMotor rightDrive;
  SteamBotMotor leftDrive;

  if (type == four_wheel || type == x_drive) {
    frontRightDrive = config.getMotors()["frontRightDrive"];
    frontLeftDrive = config.getMotors()["frontLeftDrive"];
    backRightDrive = config.getMotors()["backRightDrive"];
    backLeftDrive = config.getMotors()["backLeftDrive"];
  } else if (type == two_wheel) {
    rightDrive = config.getMotors()["rightDrive"];
    leftDrive = config.getMotors()["leftDrive"];
  } else {
    std::cout << "Failed to initialize drivetrain: Invalid DriveType: " << type
              << std::endl;
  }

  switch (type) {
  case four_wheel | x_drive:
    frontRightDrive.setVelocity(velocity);
    frontLeftDrive.setVelocity(velocity);
    backRightDrive.setVelocity(velocity);
    backLeftDrive.setVelocity(velocity);

    frontRightDrive.spinForDegrees(-(this->degreesPerAngle * angle), false);
    frontLeftDrive.spinForDegrees(this->degreesPerAngle * angle, false);
    backRightDrive.spinForDegrees(-(this->degreesPerAngle * angle), false);
    backLeftDrive.spinForDegrees(this->degreesPerAngle * angle, wait);

    break;
  case two_wheel:
    rightDrive.setVelocity(velocity);
    leftDrive.setVelocity(velocity);

    rightDrive.spinForDegrees(-(this->degreesPerAngle * angle), false);
    leftDrive.spinForDegrees(this->degreesPerAngle * angle, wait);

    break;
  default:
    std::cout << "Failed to initialize drivetrain: Invalid DriveType: " << type
              << std::endl;
  }
}

void Drivetrain::moveFor(int mm, int velocity, bool wait) {
  // TODO: Implement
}

/**
  Configuration
    Global robot configuration for other modules to depend on.
**/
Configuration::Configuration(
    std::unordered_map<std::string, SteamBotMotor> motors, DriveType type,
    int degreesPerAngle) {
  this->motors = motors;

  this->config = *new WheelConfiguration(motors, type);

  this->drivetrain = *new Drivetrain(this->config, degreesPerAngle);
}

std::unordered_map<std::string, SteamBotMotor> Configuration::getMotors() {
  return this->motors;
}

SteamBotMotor Configuration::getMotor(std::string name) {
  return this->motors[name];
}

Drivetrain Configuration::getDrivetrain() { return this->drivetrain; }

/**
  CompetitionManager
    Control the vex::competition instance and setup driver control/autonomous
    tasks.
**/
CompetitionManager::CompetitionManager(void (*driver)(), void (*auton)()) {
  this->comp.drivercontrol(driver);
  this->comp.autonomous(auton);
}

competition CompetitionManager::getCurrentCompetition() { return this->comp; }
