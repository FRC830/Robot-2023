
#include "subsystems/SwerveModule.h"
#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>


#include <frc/kinematics/SwerveDriveOdometry.h>

#include <frc/DataLogManager.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardContainer.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc2/command/CommandScheduler.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <networktables/NetworkTableValue.h>
#include <wpi/array.h>
#include <frc/kinematics/ChassisSpeeds.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <regex>
#include <string>
#include <string_view>
#include <thread>
#include <vector>

DriveSubsystem::DriveSubsystem() noexcept
{
  // Set up onboard printf-style logging.
  m_stringLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), "/DriveSubsystem/");

  // Set up the "navX" IMU first, so there's more time before it is used later.
  // See https://pdocs.kauailabs.com/navx-mxp/guidance/gyroaccelcalibration/.
  // Allow up to 20 seconds for callibration; it is supposed to be much faster,
  // when the IMU is kept still.  Consider using lights or other feedback so it
  // is very clear when this is occurring.
  DoSafeIMU("ctor", [&]() -> void
            {
    m_ahrs = std::make_unique<AHRS>(frc::SPI::Port::kMXP);

    if (!m_ahrs)
    {
      throw std::runtime_error("m_ahrs");
    }

    {
      using namespace std::chrono_literals;

      const std::chrono::steady_clock::time_point wait_until = std::chrono::steady_clock::now() + 20s;

      while (!m_ahrs->IsConnected() || m_ahrs->IsCalibrating())
      {
        std::this_thread::sleep_for(100ms);
        // Upon timeout, go on and hope for the best; the driver can switch off
        // field-relative drive if there's a major problem.
        if (std::chrono::steady_clock::now() >= wait_until)
        {
          break;
        }
      }
    } });

  ZeroHeading();

  m_orientationController = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
      pidf::kDriveThetaP,
      pidf::kDriveThetaI,
      pidf::kDriveThetaD,
      std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
          pidf::kDriveThetaMaxVelocity,
          pidf::kDriveThetaMaxAcceleration}));

  m_orientationController->EnableContinuousInput(-180.0_deg, +180.0_deg);

  // This is precomputed and used in cases where there is close to a 180 degree
  // error; needed to get things moving in some cases (rotate 180 degrees).
  m_lagrange = m_orientationController->Calculate(+177.5_deg);
  m_orientationController->Reset(0.0_deg);

  // Initial position (third parameter) defaulted to "frc::Pose2d()"; initial
  // angle (second parameter) is automatically zeroed by navX initialization.
  //m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(kDriveKinematics, botRot, 0.0_deg, GetModulePositions(), );

  // These are last, so there can be no movement from the swerve modules.
  m_frontLeftSwerveModule = std::make_unique<SwerveModule>(
      "Front Left",
      physical::kFrontLeftDriveMotorCanID,
      physical::kFrontLeftTurningMotorCanID,
      physical::kFrontLeftTurningEncoderPort,
      physical::kFrontLeftAlignmentOffset);

  m_frontRightSwerveModule = std::make_unique<SwerveModule>(
      "Front Right",
      physical::kFrontRightDriveMotorCanID,
      physical::kFrontRightTurningMotorCanID,
      physical::kFrontRightTurningEncoderPort,
      physical::kFrontRightAlignmentOffset);

  m_rearLeftSwerveModule = std::make_unique<SwerveModule>(
      "Rear Left",
      physical::kRearLeftDriveMotorCanID,
      physical::kRearLeftTurningMotorCanID,
      physical::kRearLeftTurningEncoderPort,
      physical::kRearLeftAlignmentOffset);

  m_rearRightSwerveModule = std::make_unique<SwerveModule>(
      "Rear Right",
      physical::kRearRightDriveMotorCanID,
      physical::kRearRightTurningMotorCanID,
      physical::kRearRightTurningEncoderPort,
      physical::kRearRightAlignmentOffset);

      // Initial position (third parameter) defaulted to "frc::Pose2d()"; initial
  // angle (second parameter) is automatically zeroed by navX initialization.
  m_odometry = std::make_unique<frc::SwerveDriveOdometry<4>>(kDriveKinematics, GetHeading(), GetModulePositions());
}

void DriveSubsystem::DoSafeIMU(const char *const what, std::function<void()> work) noexcept
{
  try
  {
    work();
  }
  catch (const std::exception &e)
  {
    m_ahrs = nullptr;

    std::printf("navX IMU %s exception: %s.\n", what, e.what());
  }
  catch (...)
  {
    m_ahrs = nullptr;

    std::printf("navX IMU %s unknown exception.\n", what);
  }
}

std::array<frc::SwerveModulePosition, 4> DriveSubsystem::GetModulePositions() noexcept
{
  return {
      m_frontLeftSwerveModule->GetPosition(),
      m_frontRightSwerveModule->GetPosition(),
      m_rearLeftSwerveModule->GetPosition(),
      m_rearRightSwerveModule->GetPosition()};
}

void DriveSubsystem::Periodic() noexcept
{


  m_frontLeftSwerveModule->Periodic();
  m_frontRightSwerveModule->Periodic();
  m_rearLeftSwerveModule->Periodic();
  m_rearRightSwerveModule->Periodic();

  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = -m_ahrs->GetRotation2d();
      frc::SmartDashboard::PutNumber("botRot", (double)m_ahrs->GetAngle());
    }
    else
    {
      m_theta = 0.0;

      return;
    } });

  // Compute value to apply to correct robot orientation, or to follow rotation
  // profile.  Since things wrap, 180 degrees is a sort of Lagrange Point and
  // needs special handling, using precomputed theta.
  double theta = m_orientationController->Calculate(-botRot.Degrees());
  units::angle::degree_t error = m_orientationController->GetPositionError();

  if (firstTimeI)
  {
    startingAngle = botRot.Degrees();
    firstTimeI = false;
  }


  frc::SmartDashboard::PutNumber("turning error", (double)error);
  if (error > +177.5_deg)
  {
    theta = +m_lagrange;
  }
  else if (error < -177.5_deg)
  {
    theta = -m_lagrange;
  }

  if (theta > 0.0)
  {
    theta += m_thetaF;
  }
  else if (theta < 0.0)
  {
    theta -= m_thetaF;
  }
  m_theta = theta;
  frc::SmartDashboard::PutNumber("turning error theta", (double)theta);

  // const wpi::array<frc::SwerveModulePosition, 4> states = {m_frontLeftSwerveModule->GetState(),
  //                    m_frontRightSwerveModule->GetState(), m_rearLeftSwerveModule->GetState(),
  //                    m_rearRightSwerveModule->GetState()}

  m_odometry->Update(botRot, GetModulePositions());
  frc::SmartDashboard::PutNumber("odorot", double(GetPose().Rotation().Degrees()));
  frc::SmartDashboard::PutNumber("odotrans", double(m_odometry->GetPose().Translation().X()));
  
  frc::SmartDashboard::PutNumber("odotrans", double(m_odometry->GetPose().Translation().Y()));
}

frc::Pose2d DriveSubsystem::GetPose() noexcept { return m_odometry->GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) noexcept
{
  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = -m_ahrs->GetRotation2d();
    } });

  m_odometry->ResetPosition(botRot, GetModulePositions(), pose);
}

bool DriveSubsystem::GetStatus() const noexcept
{
  return m_ahrs &&
         m_frontLeftSwerveModule->GetStatus() &&
         m_frontRightSwerveModule->GetStatus() &&
         m_rearLeftSwerveModule->GetStatus() &&
         m_rearRightSwerveModule->GetStatus();
}

void DriveSubsystem::ResetDrive() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;
  m_theta = 0.0;

  m_frontLeftSwerveModule->ResetDrive();
  m_frontRightSwerveModule->ResetDrive();
  m_rearLeftSwerveModule->ResetDrive();
  m_rearRightSwerveModule->ResetDrive();

  m_orientationController->Reset(GetHeading(), units::angular_velocity::degrees_per_second_t{GetTurnRate()});
}

void DriveSubsystem::SetDriveBrakeMode(bool brake) noexcept
{
  m_frontLeftSwerveModule->SetDriveBrakeMode(brake);
  m_frontRightSwerveModule->SetDriveBrakeMode(brake);
  m_rearLeftSwerveModule->SetDriveBrakeMode(brake);
  m_rearRightSwerveModule->SetDriveBrakeMode(brake);
}

bool DriveSubsystem::ZeroModules() noexcept { return SetTurningPosition(0.0_deg); }

bool DriveSubsystem::SetTurnInPlace() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  // Set all wheels tangent, at the given module.
  const wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0.0_mps, 0.0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0.0_mps;
  frontRight.speed = 0.0_mps;
  rearLeft.speed = 0.0_mps;
  rearRight.speed = 0.0_mps;

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  m_frontLeftSwerveModule->SetTurningPosition(frontLeft.angle.Degrees());
  m_frontRightSwerveModule->SetTurningPosition(frontRight.angle.Degrees());
  m_rearLeftSwerveModule->SetTurningPosition(rearLeft.angle.Degrees());
  m_rearRightSwerveModule->SetTurningPosition(rearRight.angle.Degrees());

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetLockWheelsX() noexcept
{
  m_rotation = 0.0;
  m_x = 0.0;
  m_y = 0.0;

  // Set all wheels at right angle to tangent, at the given module.  This forms
  // an "X", so the wheels resist being pushed (do not attempt to drive in this
  // configuration).
  const wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      frc::ChassisSpeeds{0.0_mps, 0.0_mps, physical::kMaxTurnRate});

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frontLeft.speed = 0.0_mps;
  frontRight.speed = 0.0_mps;
  rearLeft.speed = 0.0_mps;
  rearRight.speed = 0.0_mps;

  frontLeft.angle = frontLeft.angle + frc::Rotation2d(90.0_deg);
  frontRight.angle = frontRight.angle + frc::Rotation2d(90.0_deg);
  rearLeft.angle = rearLeft.angle + frc::Rotation2d(90.0_deg);
  rearRight.angle = rearRight.angle + frc::Rotation2d(90.0_deg);

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  m_frontLeftSwerveModule->SetDesiredState(frontLeft);
  m_frontRightSwerveModule->SetDesiredState(frontRight);
  m_rearLeftSwerveModule->SetDesiredState(rearLeft);
  m_rearRightSwerveModule->SetDesiredState(rearRight);

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetTurningPosition(const units::angle::degree_t position) noexcept
{
  m_rotation = 0.0;
  m_x = std::sin(units::angle::radian_t{position}.to<double>());
  m_y = std::cos(units::angle::radian_t{position}.to<double>());

  m_commandedStateFrontLeft.speed = 0.0_mps;
  m_commandedStateFrontRight.speed = 0.0_mps;
  m_commandedStateRearLeft.speed = 0.0_mps;
  m_commandedStateRearRight.speed = 0.0_mps;

  m_commandedStateFrontLeft.angle = frc::Rotation2d(position);
  m_commandedStateFrontRight.angle = frc::Rotation2d(position);
  m_commandedStateRearLeft.angle = frc::Rotation2d(position);
  m_commandedStateRearRight.angle = frc::Rotation2d(position);

  m_frontLeftSwerveModule->SetTurningPosition(position);
  m_frontRightSwerveModule->SetTurningPosition(position);
  m_rearLeftSwerveModule->SetTurningPosition(position);
  m_rearRightSwerveModule->SetTurningPosition(position);

  const bool fl = m_frontLeftSwerveModule->CheckTurningPosition();
  const bool fr = m_frontRightSwerveModule->CheckTurningPosition();
  const bool rl = m_rearLeftSwerveModule->CheckTurningPosition();
  const bool rr = m_rearRightSwerveModule->CheckTurningPosition();

  return (fl && fr && rl && rr) || !m_run;
}

bool DriveSubsystem::SetTurnToAngle(units::degree_t angle) noexcept
{
  m_orientationController->SetGoal(angle);

  if (!SetTurnInPlace())
  {
    return false;
  }

  // return SetDriveDistance(angle / 360.0_deg * physical::kDriveMetersPerTurningCircle);

  const units::meters_per_second_t linearVelocity = m_theta * physical::kMaxTurnRate / 360.0_deg * physical::kDriveMetersPerTurningCircle;

  m_frontLeftSwerveModule->SetDriveVelocity(linearVelocity);
  m_frontRightSwerveModule->SetDriveVelocity(linearVelocity);
  m_rearLeftSwerveModule->SetDriveVelocity(linearVelocity);
  m_rearRightSwerveModule->SetDriveVelocity(linearVelocity);

  return m_orientationController->AtGoal();
}

bool DriveSubsystem::SetDriveDistance(units::length::meter_t distance) noexcept
{
  m_frontLeftSwerveModule->SetDriveDistance(distance);
  m_frontRightSwerveModule->SetDriveDistance(distance);
  m_rearLeftSwerveModule->SetDriveDistance(distance);
  m_rearRightSwerveModule->SetDriveDistance(distance);

  const bool fl = m_frontLeftSwerveModule->CheckDriveDistance();
  const bool fr = m_frontRightSwerveModule->CheckDriveDistance();
  const bool rl = m_rearLeftSwerveModule->CheckDriveDistance();
  const bool rr = m_rearRightSwerveModule->CheckDriveDistance();

  return (fl && fr && rl && rr) || !m_run;
}

void DriveSubsystem::shuffAngles()
{
  
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed, units::radians_per_second_t rot,
                           bool fieldRelative, units::meter_t x_center, units::meter_t y_center) noexcept
{
  m_rotation = rot / physical::kMaxTurnRate;
  m_x = xSpeed / physical::kMaxDriveSpeed;
  m_y = ySpeed / physical::kMaxDriveSpeed;



  frc::Rotation2d botRot;

  DoSafeIMU("GetRotation2d()", [&]() -> void
            {
    if (m_ahrs)
    {
      botRot = m_ahrs->GetRotation2d();
    } });

  if (!m_ahrs)
  {
    fieldRelative = false;
  }

  //rot = rot + (physical::kMaxTurnRate* (int)m_theta/30);

  // Center of rotation argument is defaulted to the center of the robot above,
  // but it is also possible to rotate about a different point.



  if (rot != units::angular_velocity::radians_per_second_t(0))
  {
    frc::SmartDashboard::PutBoolean("Is turning", true);
    frc::SmartDashboard::PutNumber("targetRot", (double)targetRot.Degrees());
    targetRot = botRot;
  }
  else{
    frc::SmartDashboard::PutBoolean("Is turning", false);
    
    frc::SmartDashboard::PutNumber("targetRot", (double)targetRot.Degrees());
  }
 
 
  wpi::array<frc::SwerveModuleState, 4> states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeed, ySpeed, rot, botRot)
                    : frc::ChassisSpeeds{xSpeed, ySpeed, rot},
      frc::Translation2d(x_center, y_center));



  kDriveKinematics.DesaturateWheelSpeeds(&states, physical::kMaxDriveSpeed);

  //m_orientationController->SetGoal(targetRot.Degrees());
  
  SetModuleStates(states);

  auto [frontLeft, frontRight, rearLeft, rearRight] = states;

  frc::SmartDashboard::PutNumber("frontLeft state",double(frontLeft.angle.Degrees()));
  frc::SmartDashboard::PutNumber("frontRight state",double(frontRight.angle.Degrees()));
  frc::SmartDashboard::PutNumber("rearLeft state",double(rearLeft.angle.Degrees()));
  frc::SmartDashboard::PutNumber("rearRight state",double(rearRight.angle.Degrees()));

}

void DriveSubsystem::ResetEncoders() noexcept
{
  m_frontLeftSwerveModule->ResetEncoders();
  m_frontRightSwerveModule->ResetEncoders();
  m_rearLeftSwerveModule->ResetEncoders();
  m_rearRightSwerveModule->ResetEncoders();
}

void DriveSubsystem::SetModuleStates(wpi::array<frc::SwerveModuleState, 4> &desiredStates) noexcept
{
  auto [frontLeft, frontRight, rearLeft, rearRight] = desiredStates;

  m_commandedStateFrontLeft = frontLeft;
  m_commandedStateFrontRight = frontRight;
  m_commandedStateRearLeft = rearLeft;
  m_commandedStateRearRight = rearRight;

  // Don't command turning if there is no drive; this is used from Drive(), and
  // it winds up causing the modules to all home to zero any time there is no
  // joystick input.  This check causes them to stay where they are, which is
  // no worse and saves energy, wear, and, potentially, time.  This is done
  // before applying m_limit (intentionally).
  if (frontLeft.speed == 0.0_mps &&
      frontRight.speed == 0.0_mps &&
      rearLeft.speed == 0.0_mps &&
      rearRight.speed == 0.0_mps)
  {
    m_frontLeftSwerveModule->SetDriveVelocity(0.0_mps);
    m_frontRightSwerveModule->SetDriveVelocity(0.0_mps);
    m_rearLeftSwerveModule->SetDriveVelocity(0.0_mps);
    m_rearRightSwerveModule->SetDriveVelocity(0.0_mps);

    return;
  }

  // m_limit is always unity, except in Test Mode.  So, by default, it does not
  // modify anything here.  In Test Mode, it can be used to slow things down.
  frontLeft.speed *= m_limit;
  frontRight.speed *= m_limit;
  rearLeft.speed *= m_limit;
  rearRight.speed *= m_limit;

  // frontLeft.angle = frontLeft.angle + frc::Rotation2d(55.0_deg);
  // frontRight.angle = frontRight.angle + frc::Rotation2d(205.0_deg);
  // rearLeft.angle = rearLeft.angle + frc::Rotation2d(0.0_deg);
  // rearRight.angle = rearRight.angle + frc::Rotation2d(100.0_deg);

  frc::SmartDashboard::PutNumber("frontLeft.GetTurningPosition()",double(m_frontLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("frontRight.GetTurningPosition()",double(m_frontRightSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("rearLeft.GetTurningPosition()",double(m_rearLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("rearRight.GetTurningPosition()",double(m_rearRightSwerveModule->GetTurningPosition()));

  

  // To avoid brownout, reduce power when wheels are too far out of position.
  const units::angle::degree_t frontLeftTurningError = m_frontLeftSwerveModule->GetTurningPosition() - frontLeft.angle.Degrees();
  const units::angle::degree_t frontRightTurningError = m_frontRightSwerveModule->GetTurningPosition() - frontRight.angle.Degrees();
  const units::angle::degree_t rearLeftTurningError = m_rearLeftSwerveModule->GetTurningPosition() - rearLeft.angle.Degrees();
  const units::angle::degree_t rearRightTurningError = m_rearRightSwerveModule->GetTurningPosition() - rearRight.angle.Degrees();
  const double totalTurningError = std::abs(frontLeftTurningError.to<double>()) +
                                   std::abs(frontRightTurningError.to<double>()) +
                                   std::abs(rearLeftTurningError.to<double>()) +
                                   std::abs(rearRightTurningError.to<double>());

  if (totalTurningError > 10.0)
  {
    frontLeft.speed *= 0.5;
    frontRight.speed *= 0.5;
    rearLeft.speed *= 0.5;
    rearRight.speed *= 0.5;
  }

  m_frontLeftSwerveModule->SetDesiredState(frontLeft);
  m_frontRightSwerveModule->SetDesiredState(frontRight);
  m_rearLeftSwerveModule->SetDesiredState(rearLeft);
  m_rearRightSwerveModule->SetDesiredState(rearRight);
}

void DriveSubsystem::OutputWheelPositions()
{
  frc::SmartDashboard::PutNumber("frontLeft.GetTurningPosition()", double(m_frontLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("frontRight.GetTurningPosition()", double(m_frontRightSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("rearLeft.GetTurningPosition()", double(m_rearLeftSwerveModule->GetTurningPosition()));
  frc::SmartDashboard::PutNumber("rearRight.GetTurningPosition()", double(m_rearRightSwerveModule->GetTurningPosition()));
}

units::degree_t DriveSubsystem::GetHeading() noexcept
{
  units::degree_t heading{0.0};

  DoSafeIMU("GetAngle()", [&]() -> void
            {
    if (m_ahrs)
    {
      heading = units::degree_t{-m_ahrs->GetAngle()}; // In degrees already.
    } });

  return heading;
}

void DriveSubsystem::ZeroHeading() noexcept
{
  DoSafeIMU("ZeroYaw()", [&]() -> void
            {
    if (m_ahrs)
    {
      m_ahrs->ZeroYaw();
    } });

    targetRot = frc::Rotation2d(0.0_deg);
}

double DriveSubsystem::GetTurnRate() noexcept
{
  double rate{0.0};

  DoSafeIMU("GetRate()", [&]() -> void
            {
    if (m_ahrs)
    {
      rate = -m_ahrs->GetRate(); // In degrees/second (units not used in WPILib).
    } });

  return rate;
}

void DriveSubsystem::ThetaPID(double P, double I, double D, double F, double V, double A) noexcept
{
  m_thetaF = F;

  m_orientationController->SetPID(P, I, D);
  //m_orientationController->SetConstraints(std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
  //    units::angular_velocity::degrees_per_second_t{V},
  //    units::angular_acceleration::degrees_per_second_squared_t{A}}));
}

void DriveSubsystem::BurnConfig() noexcept
{
  m_frontLeftSwerveModule->BurnConfig();
  m_frontRightSwerveModule->BurnConfig();
  m_rearLeftSwerveModule->BurnConfig();
  m_rearRightSwerveModule->BurnConfig();
}

void DriveSubsystem::ClearFaults() noexcept
{
  m_frontLeftSwerveModule->ClearFaults();
  m_frontRightSwerveModule->ClearFaults();
  m_rearLeftSwerveModule->ClearFaults();
  m_rearRightSwerveModule->ClearFaults();
}

void DriveSubsystem::Align() noexcept
{
  SetTurnToAngle(startingAngle);
}