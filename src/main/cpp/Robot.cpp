// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "Robot.h"

#include <frc/DataLogManager.h>
#include <frc/DriverStation.h>
#include <frc/livewindow/LiveWindow.h>
#include <frc2/command/CommandScheduler.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/event/EventLoop.h>

#include "Constants.h"

#include <frc2/command/button/JoystickButton.h>
#include <frc2/command/button/POVButton.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandScheduler.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/RunCommand.h>
#include <frc/Preferences.h>


#include <cmath>
#include <cstdio>
#include <string>


void Robot::RobotInit() noexcept
{
  frc::LiveWindow::SetEnabled(false);
  frc::LiveWindow::DisableAllTelemetry();

  frc::DataLogManager::Start();
  frc::DriverStation::StartDataLog(frc::DataLogManager::GetLog());

  
  //create a feild for realignment
  frc::SmartDashboard::PutNumber("0 if realign", 1);


  // Create feilds in the persistant preferences for all the wheel alignment offsets if they don't already exist
  // This will not override any existing preferences
  frc::Preferences::InitDouble(physical::kFrontLeftAlignmentOffsetKey, physical::kFrontLeftAlignmentOffset);
  frc::Preferences::InitDouble(physical::kFrontRightAlignmentOffsetKey, physical::kFrontRightAlignmentOffset);
  frc::Preferences::InitDouble(physical::kRearLeftAlignmentOffsetKey, physical::kRearLeftAlignmentOffset);
  frc::Preferences::InitDouble(physical::kRearRightAlignmentOffsetKey, physical::kRearRightAlignmentOffset);
  
  // Read from the preferences and set the alignment offests to those values
  physical::kFrontLeftAlignmentOffset = frc::Preferences::GetDouble(physical::kFrontLeftAlignmentOffsetKey);
  physical::kFrontRightAlignmentOffset = frc::Preferences::GetDouble(physical::kFrontRightAlignmentOffsetKey);
  physical::kRearLeftAlignmentOffset = frc::Preferences::GetDouble(physical::kRearLeftAlignmentOffsetKey);
  physical::kRearRightAlignmentOffset = frc::Preferences::GetDouble(physical::kRearRightAlignmentOffsetKey);



  // Initialize all of your commands and subsystems here
  //frc2::Command::Initialize();


  // Configure the button bindings
  ConfigureButtonBindings();

  // Set up default drive command; non-owning pointer is passed by value.
  auto driveRequirements = {dynamic_cast<frc2::Subsystem *>(&m_driveSubsystem)};
  auto subsystemRequirements = {dynamic_cast<frc2::Subsystem *>(&m_subsystems)};


  // Drive, as commanded by operator joystick controls.
  m_driveCommand = std::make_unique<frc2::RunCommand>(
      [&]() -> void
      {
        if (m_lock)
        {
          (void)m_driveSubsystem.SetLockWheelsX();

          return;
        }

        const auto controls = GetDriveTeleopControls();

        

        m_driveSubsystem.Drive(
            std::get<0>(controls) * physical::kMaxDriveSpeed,
            std::get<1>(controls) * physical::kMaxDriveSpeed,
            std::get<2>(controls) * physical::kMaxTurnRate,
            std::get<3>(controls));

        m_subsystems.SetArmPIDF(pidf::kArmP, pidf::kArmI, pidf::kArmD);

      },
      driveRequirements);
  // autonCommand = std::make_unique<frc2::RunCommand>(
  //     [&]() -> void
  //     {
  //     wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates();
  //     frc::ChassisSpeeds::FromFieldRelativeSpeeds(0, 0.1, 0, botRot),
  //     },
  //     driveRequirements);

  // Point swerve modules, but do not actually drive.

  autonChooser.AddOption("taxi", 0); 
  //autonChooser.addOption("taxi", 0);

  
  m_subsystems.SubsystemsInit();

}

/**
 * This function is called every robot packet, no matter the mode. Use
 * this for items like diagnostics that you want to run during disabled,
 * autonomous, teleoperated and test.
 *
 * <p> This runs after the mode specific periodic functions, but before
 * LiveWindow and SmartDashboard integrated updating.
 */
// This performs the sense portion of sense/think/act, including sending test
// mode telemetry.  It also handles think and act, except in test mode.  In the
// latter case, TestPeriodic() handles manually driven act.
void Robot::RobotPeriodic() noexcept
{
  
  frc2::CommandScheduler::GetInstance().Run();

  
  frc::SmartDashboard::PutNumber("Front Left Turning Position", double(m_driveSubsystem.m_frontLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("Front Right Turning Position", double(m_driveSubsystem.m_frontRightSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("Back Left Turning Position", double(m_driveSubsystem.m_rearLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("Back Right Turning Position", double(m_driveSubsystem.m_rearRightSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("Front Left Drive Position", double(m_driveSubsystem.m_frontLeftSwerveModule->GetDriveVelocity()));
  frc::SmartDashboard::PutNumber("Front Right Drive Position", double(m_driveSubsystem.m_frontRightSwerveModule->GetDriveVelocity()));
  frc::SmartDashboard::PutNumber("Back Left Drive Position", double(m_driveSubsystem.m_rearLeftSwerveModule->GetDriveVelocity()));
  frc::SmartDashboard::PutNumber("Back Right Drive Position", double(m_driveSubsystem.m_rearRightSwerveModule->GetDriveVelocity()));

  
  frc::SmartDashboard::PutNumber("Front Left Target PID", double(m_driveSubsystem.m_frontLeftSwerveModule->m_rioPIDController->GetSetpoint().position));
  frc::SmartDashboard::PutNumber("Front Right Target PID", double(m_driveSubsystem.m_frontRightSwerveModule->m_rioPIDController->GetSetpoint().position));
  frc::SmartDashboard::PutNumber("Back Left Target PID", double(m_driveSubsystem.m_rearLeftSwerveModule->m_rioPIDController->GetSetpoint().position));
  frc::SmartDashboard::PutNumber("Back Right Target PID", double(m_driveSubsystem.m_rearRightSwerveModule->m_rioPIDController->GetSetpoint().position));

  m_driveSubsystem.OutputWheelPositions();
  
  if (frc::SmartDashboard::GetNumber("0 if realign", 1) == 0)
  {
    physical::kFrontLeftAlignmentOffset += -int(m_driveSubsystem.m_frontLeftSwerveModule->GetTurningPosition());
    physical::kFrontRightAlignmentOffset += -int(m_driveSubsystem.m_frontRightSwerveModule->GetTurningPosition());
    physical::kRearLeftAlignmentOffset += -int(m_driveSubsystem.m_rearLeftSwerveModule->GetTurningPosition());
    physical::kRearRightAlignmentOffset += -int(m_driveSubsystem.m_rearRightSwerveModule->GetTurningPosition());

    frc::Preferences::SetDouble(physical::kFrontLeftAlignmentOffsetKey, physical::kFrontLeftAlignmentOffset);
    frc::Preferences::SetDouble(physical::kFrontRightAlignmentOffsetKey, physical::kFrontRightAlignmentOffset);
    frc::Preferences::SetDouble(physical::kRearLeftAlignmentOffsetKey, physical::kRearLeftAlignmentOffset);
    frc::Preferences::SetDouble(physical::kRearRightAlignmentOffsetKey, physical::kRearRightAlignmentOffset);
  
    frc::SmartDashboard::PutNumber("0 if realign", 1);
  }
  
  // frc::SmartDashboard::PutNumber("Robot Periodic", 1.0);

  // DriveSubsystem::shuffAngles();

  eventLoop.Poll();

  m_subsystems.SubsystemsPeriodic();
  //m_subsystems.SetArmPIDF(frc::SmartDashboard::PutNumber("ArmP", pidf::kArmP), frc::SmartDashboard::PutNumber("ArmI", pidf::kArmI), frc::SmartDashboard::PutNumber("ArmD", pidf::kArmD));
}

/**
 * This function is called once each time the robot enters Disabled mode. You
 * can use it to reset any subsystem information you want to clear when the
 * robot is disabled.
 */

void Robot::DisabledPeriodic() noexcept {}


/**
 * This autonomous runs the autonomous command selected by your {@link
 * RobotContainer} class.
 */
void Robot::AutonomousInit() noexcept
{
  
  //m_driveSubsystem.SetDefaultCommand(*m_driveCommand);
}

void Robot::AutonomousPeriodic() noexcept {

  if (counter < 300)
  {
    m_driveSubsystem.Drive(1_mps, 0_mps, 0_deg_per_s, false);
    counter += 1;
  }
  else
  {
    m_driveSubsystem.Drive(0_mps, 0_mps, 0_deg_per_s, false);
  }

  //m_auton.runAuton(0);
  //m_auton.runAuton(autonChooser.GetSelected(), &m_driveSubsystem);
  
}
  // Scheduler::GetInstance()->Run();
void Robot::AutonomousExit() noexcept {}

void Robot::TeleopInit() noexcept {

  m_driveSubsystem.ClearFaults();

  m_driveSubsystem.ResetEncoders();

  m_driveSubsystem.SetDefaultCommand(*m_driveCommand);
}

/**
 * This function is called periodically during operator control.
 */
void Robot::TeleopPeriodic() noexcept {
    if (m_xbox.GetRightTriggerAxis() > 0.4)
  {
    frc::SmartDashboard::PutNumber("arm", 400);
    std::cout << "arm up" << std::endl;
    m_subsystems.RotateArm(true);
  }
  else if (m_xbox.GetLeftTriggerAxis() > 0.4)
  {
    frc::SmartDashboard::PutNumber("arm", 300);
    std::cout << "arm down" << std::endl;
    m_subsystems.RotateArm(false);
  }

  //m_xbox.LeftTrigger(0.4, &eventLoop).IfHigh([&]() -> void  {m_subsystems.RotateArm(false);});

  // m_xbox.POVUp(&eventLoop).IfHigh([&]() -> void  {m_lock = true;});
  // m_xbox.POVDown(&eventLoop).IfHigh([&]() -> void  {m_lock = false;});

  if (m_xbox.GetPOV() > 0 && m_xbox.GetPOV() < 180)
  {
    m_lock = true;
  }
  else if (m_xbox.GetPOV() > 180 && m_xbox.GetPOV() < 360)
  {
    m_lock = false;
  }
}

void Robot::TeleopExit() noexcept {}

void Robot::TestInit() noexcept {}

/**
 * This function is called periodically during test mode.
 * Note that test mode does not follow the command-based paradigm; it follows
 * Init/Periodic.
 */
void Robot::TestPeriodic() noexcept
{
}

void Robot::TestExit() noexcept {}


void Robot::ConfigureButtonBindings() noexcept
{
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kA).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_slow = !m_slow; },
                                                                                                  {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kB).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_subsystems.ToggleGrabberPnumatics(); },
                                                                                                  {&m_subsystems}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kStart).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  { m_driveSubsystem.ZeroHeading(); m_fieldOriented = true; },
                                                                                                  {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kBack).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                                                  {  m_fieldOriented = false; },
                                                                                                  {}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kX).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                    
                                                                                                  { m_subsystems.SetGrabberWheels(true); },
                                                                                                  {&m_subsystems}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kY).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                    
                                                                                                  {  m_subsystems.SetGrabberWheels(false); },
                                                                                                  {&m_subsystems}));

  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kLeftBumper).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                    
                                                                                                  { m_subsystems.moveTelescopethingy(false); },
                                                                                                  {&m_subsystems}));
  frc2::JoystickButton(&m_xbox, frc::XboxController::Button::kRightBumper).WhenPressed(frc2::InstantCommand([&]() -> void
                                                                    
                                                                                                  { m_subsystems.moveTelescopethingy(true); },
                                                                                                  {&m_subsystems}));
  
  //m_xbox.RightTrigger(0.4, &eventLoop).IfHigh([&]() -> void  {m_subsystems.RotateArm(true);});


  //telescope 

  // m_xbox.LeftBumper(&eventLoop).IfHigh([&]() -> void  {m_subsystems.moveTelescopethingy(true);});
  // m_xbox.RightBumper(&eventLoop).IfHigh([&]() -> void  {m_subsystems.moveTelescopethingy(false);});



  // m_xbox.POVLeft(&eventLoop).IfHigh([&]() -> void  {m_subsystems.RotateArm(false);});
  // m_xbox.POVRight(&eventLoop).IfHigh([&]() -> void  {m_subsystems.RotateArm(false);});


 }



std::tuple<double, double, double, bool> Robot::GetDriveTeleopControls() noexcept
{
  double x = -m_xbox.GetLeftY();
  double y = -m_xbox.GetLeftX();
  double z = -m_xbox.GetRightX();

  // between out = in^3.0 and out = in.
  auto shape = [](double raw, double mixer = 0.75) -> double
  {
    // Input deadband around 0.0 (+/- range).
    constexpr double range = 0.05;

    constexpr double slope = 1.0 / (1.0 - range);

    if (raw >= -range && raw <= +range)
    {
      raw = 0.0;
    }
    else if (raw < -range)
    {
      raw += range;
      raw *= slope;
    }
    else if (raw > +range)
    {
      raw -= range;
      raw *= slope;
    }

    return mixer * std::pow(raw, 3.0) + (1.0 - mixer) * raw;
  };

  x = shape(x);
  y = shape(y);
  z = shape(z, 0.0);

  if (m_slow)
  {
    x *= 0.35;
    y *= 0.35;
    z *= 0.40;
  }
  else
  { // XXX Still needed?
    x *= abs(x) * 0.75;
    y *= abs(y) * 0.75;
    z *= 1.6;
  }

  if (abs(x) > 0.1 || abs(y) > 0.1 || abs(z) > 0.1)
  {
    m_lock = false;
  }

  return std::make_tuple(x, y, z, m_fieldOriented);
}

#ifndef RUNNING_FRC_TESTS
int main()
{
  return frc::StartRobot<Robot>();
}
#endif
