// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include <frc/GenericHID.h>
#include <frc/XboxController.h>
#include <frc2/command/RunCommand.h>
#include <frc/event/BooleanEvent.h>

#include <frc/AnalogEncoder.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Auton.h"

#include "subsystems/Subsystems.h"

#include <frc/smartdashboard/SendableChooser.h>


#include <memory>
#include <tuple>

class Robot : public frc::TimedRobot
{
public:
  void RobotInit() noexcept override;
  void RobotPeriodic() noexcept override;
  // void DisabledInit() noexcept override;
  void DisabledPeriodic() noexcept override;
  //void DisabledExit() noexcept override;
  void AutonomousInit() noexcept override;
  void AutonomousPeriodic() noexcept override;
  void AutonomousExit() noexcept override;
  void TeleopInit() noexcept override;
  void TeleopPeriodic() noexcept override;
  void TeleopExit() noexcept override;
  void TestInit() noexcept override;
  void TestPeriodic() noexcept override;
  void TestExit() noexcept override;

  void GetDashBoardValues();
  void PutDashBoardValues();

private:
  // Have it null by default so that if testing teleop it
  // doesn't have undefined behavior and potentially crash.
  frc2::Command *m_autonomousCommand{nullptr};

  std::tuple<double, double, double, bool> GetDriveTeleopControls() noexcept;

  void ConfigureButtonBindings() noexcept;

  bool m_fieldOriented{false};
  bool m_lock{false};
  bool m_slow{false};

  // The robot's subsystems and commands are defined here...
  DriveSubsystem m_driveSubsystem;
  Subsystems m_subsystems;

  Auton m_auton;

  int counter = 0;
  int c1 = 275;
  int c2 = 325;
  int c3 = 375;
  int c4 = 425;
  int c5 = 475;


  double autonBalanceSpeed = 1;
  double autonDeadzoneAngle = 10;

  std::unique_ptr<frc2::RunCommand> m_driveCommand;
  std::unique_ptr<frc2::RunCommand> m_pointCommand;
  
  frc::XboxController m_Pilot{0};
  frc::XboxController m_Copilot{1};
  frc::EventLoop eventLoop;
  frc::GenericHID m_buttonBoard{1};

  frc::SendableChooser<int> autonChooser;

  bool turbo = false;

};