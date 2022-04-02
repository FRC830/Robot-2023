// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutonomousCommands.h"

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/WaitCommand.h>

AutonomousCommand::AutonomousCommand(DriveSubsystem *const drive, FeederSubsystem *const feeder, InfrastructureSubsystem *const infrastructure, ShooterSubsystem *const shooter) noexcept
    : m_drive{drive}, m_feeder{feeder}, m_infrastructure{infrastructure}, m_shooter{shooter} { SetName("Autonomous"); }

void AutonomousCommand::Initialize() noexcept
{
    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);
    m_feeder->LockIntake();
    m_feeder->RaiseIntake();

    m_shooter->Stop();

    FPGATime_ = frc::RobotController::GetFPGATime();
    counter_ = 0;
}

void AutonomousCommand::Execute() noexcept
{
    const uint64_t FPGATime = frc::RobotController::GetFPGATime();
    const uint deltaTime = (FPGATime - FPGATime_) / 1000; // ms

    if (deltaTime < 100) // 100ms
    {
        return;
    }

    FPGATime_ = FPGATime;

    // Will arrive here every 100ms, for 15s autonomous period.

    const units::pressure::pounds_per_square_inch_t pressure = m_infrastructure->GetPressure();

    if (pressure >= 75_psi)
    {
        printf("Auto Stage 0.\n");

        pressure_ = true;
    }

    if (!pressure_)
    {
        return;
    }

    ++counter_;

    // Assuming pneumatics were prefilled, counter runs 1 - ~150.

    // Sit still, run intake/elevator, spin up shooter.
    if (counter_ <= 5) // 0.0 - 0.5s
    {
        if (counter_ == 5)
        {
            printf("Auto Stage 1.\n");
        }

        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(1.0);
        m_feeder->LockIntake();
        m_feeder->RaiseIntake();

        m_shooter->Default(1.0, 575.0);

        return;
    }

    // Drop intake (first ball is preloaded).
    if (counter_ <= 10) // 0.5 - 1.0s
    {
        if (counter_ == 10)
        {
            printf("Auto Stage 2.\n");
        }

        m_feeder->DropIntake();

        return;
    }

    // Reextend drop catches, lower intake.
    if (counter_ <= 15) // 1.0 - 1.5s
    {
        if (counter_ == 15)
        {
            printf("Auto Stage 3.\n");
        }

        m_feeder->LockIntake();
        m_feeder->LowerIntake();

        return;
    }

    // Take first shot!
    if (counter_ <= 35) // 1.5 - 3.5s
    {
        if (counter_ == 35)
        {
            printf("Auto Stage 4.\n");
        }

        m_feeder->Fire();

        return;
    }

    // Back up, also stop feeder and shooter.
    if (counter_ <= 55) // 3.5 - 5.5s
    {
        if (counter_ == 55)
        {
            printf("Auto Stage 5.\n");
        }

        m_drive->Drive(-0.55_mps, 0_mps, 0_deg_per_s, false);

        m_feeder->Default(0.0);

        m_shooter->Stop();

        return;
    }

    // Stop.
    if (counter_ <= 60) // 5.5 - 6.0s
    {
        if (counter_ == 60)
        {
            printf("Auto Stage 6.\n");
        }

        m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

        return;
    }

    // Turn around.
    if (counter_ <= 100) // 6.0 - 10s
    {
        if (counter_ == 100)
        {
            printf("Auto Stage 7.\n");
        }

        (void)m_drive->SetTurnToAngle(180_deg);

        return;
    }

    finished_ = true;
}

void AutonomousCommand::End(bool interrupted) noexcept
{
    printf("Auto Final: %u.\n", counter_);

    m_drive->Drive(0_mps, 0_mps, 0_deg_per_s, false);

    m_feeder->Default(0.0);

    m_shooter->Stop();
}
