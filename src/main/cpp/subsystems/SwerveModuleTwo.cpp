#include "subSystems/SwerveModuleTwo.h"

#include <frc/DataLogManager.h>
#include <frc/RobotController.h>
#include <frc/shuffleboard/BuiltInWidgets.h>
#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/shuffleboard/ShuffleboardLayout.h>
#include <frc/shuffleboard/ShuffleboardTab.h>
#include <frc/shuffleboard/SimpleWidget.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTableValue.h>
#include <units/voltage.h>
#include <wpi/StringMap.h>

#include <SparkMaxRelativeEncoder.h>

#include <bitset>
#include <cmath>
#include <cstdio>
#include <exception>
#include <ios>
#include <iomanip>
#include <sstream>
#include <string>

#include <iostream>

units::angle::degree_t GetTurningPosition() noexcept;


SwerveModuleTwo::SwerveModuleTwo(
    const char *const name,
    const int driveMotorCanID,
    const int turningMotorCanID,
    const int turningEncoderPort,
    const int alignmentOffset) noexcept : m_name{name}
{
    // Set up onboard printf-style logging.
    std::string logName{"/SwerveModule/"};
    logName += name;
    m_stringLog = wpi::log::StringLogEntry(frc::DataLogManager::GetLog(), logName);

    std::printf("Swerve Module (%s) Initialization... ", m_name.c_str());

    // Construct turning position PID controller on the roboRIO; only used when
    // turning position control is running on the roborRIO.
    m_rioPIDController = std::make_unique<frc::ProfiledPIDController<units::angle::degrees>>(
        pidf::kTurningPositionP,
        pidf::kTurningPositionI,
        pidf::kTurningPositionD,
        std::move(frc::TrapezoidProfile<units::angle::degrees>::Constraints{
            pidf::kTurningPositionMaxVelocity,
            pidf::kTurningPositionMaxAcceleration}));

    m_rioPIDController->EnableContinuousInput(0.0_deg, +360.0_deg);

    // Construct turning absolute duty cycle encoder.  A `DigitalSource` is
    // required by the `DutyCycle` ctor.  Nothing here is expected to fail.
    // Note this has no explicit SetInverted() method; flip the turning motor
    // and it's incremental encoder in order to make things work out/line up.
    m_turningPositionPWM = std::make_unique<>(turningEncoderPort);
    encoderAlignmentOffset = alignmentOffset;

    // Motor controller configurations are only checked (or saved) in test mode
    // but a minimal amount is set up in these methods.
    m_turningMotorBase = SparkMaxFactory::CreateSparkMax(m_name + std::string(" Turning"), turningMotorCanID, m_turningMotorInverted);
    m_turningMotor = std::make_unique<SmartMotor<units::angle::degrees>>(*m_turningMotorBase);

    m_driveMotorBase = SparkMaxFactory::CreateSparkMax(m_name + std::string(" Drive"), driveMotorCanID, m_driveMotorInverted);
    m_driveMotor = std::make_unique<SmartMotor<units::length::meters>>(*m_driveMotorBase);

    // kStatus1 includes velocity; kStatus2 includes position -- these are made
    // more frequent when graphing but may normally have a longer period.

    // Motor is in turns and turns/minute (RPM), do degrees and degrees/second.
    m_turningMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{360.0}},
        {"kVelocityConversionFactor", double{360.0 / 60.0}},
    });
    m_turningMotor->ApplyConfig(false);

    // Motor is in turns and turns/minute (RPM), do meters and meters/second.
    m_driveMotor->AddConfig(SmartMotorBase::ConfigMap{
        {"kStatus1", uint{250}}, // ms
        {"kStatus2", uint{250}}, // ms
        {"kPositionConversionFactor", double{physical::kDriveMetersPerRotation}},
        {"kVelocityConversionFactor", double{physical::kDriveMetersPerRotation / 60.0}},
    });
    m_driveMotor->ApplyConfig(false);

    std::printf(" OK.\n");
}

void SwerveModuleTwo::ResetTurning()
{
     const std::optional<units::angle::degree_t> position = units::degree_t(360*(m_turningPositionPWM->GetAbsolutePosition() - m_turningPositionPWM->GetPositionOffset()) + encoderAlignmentOffset);

    // If absolute position isn't available, there's no basis for resetting and
    // it's better to just let any old data stand.  This is when manual
    // pointing of the modules before a match could ensure a reasonable zero.
    // Of course, if anything is reset, etc. all bets are off -- still, leaving
    // things alone is the best that can be done under these circumstances.
    if (!position.has_value())
    {
        return;
    }

    m_turningPosition = position.value();
    m_turningMotor->SpecifyPosition(m_turningPosition);

    m_rioPIDController->Reset(m_turningPosition);
}
void SetDesiredState(const frc::SwerveModuleState &referenceState);


