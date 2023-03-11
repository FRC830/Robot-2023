#include "../include/subsystems/Subsystems.h"
#include <iostream>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/controller/PIDController.h>

Subsystems::Subsystems() : ArmPIDController (pidf::kArmP, pidf::kArmI, pidf::kArmD), TelePIDController (pidf::kTeleP, pidf::kTeleI, pidf::kTeleD) 
{
}

void Subsystems::SubsystemsInit()
{
  frc::SmartDashboard::PutNumber("ArmPCoefficient", pidf::kArmP);
  frc::SmartDashboard::PutNumber("ArmPAdder", 0);
  frc::SmartDashboard::PutNumber("ArmI", pidf::kArmI);
  frc::SmartDashboard::PutNumber("ArmD", pidf::kArmD);
  ArmMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  frc::SmartDashboard::PutNumber("TeleP", pidf::kTeleP);
  frc::SmartDashboard::PutNumber("TeleI", pidf::kTeleI);
  frc::SmartDashboard::PutNumber("TeleD", pidf::kTeleD);
  teleMotor.SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);

  frc::SmartDashboard::PutBoolean("Set this to True if Set Tele PID Target", false);
  frc::SmartDashboard::PutBoolean("Set this to True if Set Arm PID Target", false);

  frc::SmartDashboard::PutNumber("Arm PID target to Set to for manual setting", 0);
  frc::SmartDashboard::PutNumber("Tele PID target to Set to for manual setting", 0);
    
}

void Subsystems::SubsystemsPeriodic()
{
    ArmMotor.Set(ArmPIDController.Calculate(GetArmEncoderAngle()));
    SetArmPIDF
    (
        /*cos(GetArmEncoderAngle()*(3.14 /180)) * TeleMotorEncoder.GetPosition() * 
            */frc::SmartDashboard::GetNumber("ArmPCoefficient", pidf::kArmP) /*+/*
            /*frc::SmartDashboard::GetNumber("ArmPAdder", 0)*/, 
        frc::SmartDashboard::GetNumber("ArmI", pidf::kArmI), 
        frc::SmartDashboard::GetNumber("ArmD", pidf::kArmD)
    );

    teleMotor.Set(TelePIDController.Calculate(TeleMotorEncoder.GetPosition()));
    SetTelePIDF
    (
        frc::SmartDashboard::GetNumber("TeleP", pidf::kTeleP), 
        frc::SmartDashboard::GetNumber("TeleI", pidf::kTeleI), 
        frc::SmartDashboard::GetNumber("TeleD", pidf::kTeleD)
    );

    bool settingArm = frc::SmartDashboard::GetBoolean("Set this to True if Set Arm PID Target", false);
    bool settingTele = frc::SmartDashboard::GetBoolean("Set this to True if Set Tele PID Target", false);

    if (settingArm)
    {
        SetArmPIDTarget (frc::SmartDashboard::GetNumber("Arm PID target to Set to for manual setting", GetArmEncoderAngle())); 
    }
    if (settingTele)
    {
        SetTelePIDTarget (frc::SmartDashboard::GetNumber("Arm PID target to Set to for manual setting", TeleMotorEncoder.GetPosition()));
    }

    frc::SmartDashboard::PutBoolean("Set this to True if Set Tele PID Target", false);
    frc::SmartDashboard::PutBoolean("Set this to True if Set Arm PID Target", false);


    frc::SmartDashboard::PutNumber("Arm Position", GetArmEncoderAngle());
    frc::SmartDashboard::PutNumber("Tele Position", TeleMotorEncoder.GetPosition());
    frc::SmartDashboard::PutNumber("Arm PID Target", ArmPIDController.GetSetpoint());
    frc::SmartDashboard::PutNumber("Tele PID Target", TelePIDController.GetSetpoint());
}

void Subsystems::SetGrabberWheels(bool direction)
{
    GrabberWheelsMotor.Set(GrabberWheelSpeeds *-1);
}

void Subsystems::DisableGrabberWheels()
{
    GrabberWheelsMotor.Set(GrabberWheelSpeeds *-1);
}
void Subsystems::ToggleGrabberPnumatics()
{
    if (GrabberOnOff)
    {
        GrabberSolenoid.Set(true);
    }
    else
    {
        GrabberSolenoid.Set(false);
    }

    GrabberOnOff = !GrabberOnOff;

}
void Subsystems::SetArmPIDF(double p, double i, double d)
{
    ArmPIDController.SetP(p);
    ArmPIDController.SetI(i);
    ArmPIDController.SetD(d);
    //ArmPIDController.SetF(f);
}

void Subsystems::SetTelePIDF(double p, double i, double d)
{
    TelePIDController.SetP(p);
    TelePIDController.SetI(i);
    TelePIDController.SetD(d);
    //ArmPIDController.SetF(f);
}

void Subsystems::RotateArm(bool direction)
{
    auto setpoint = ArmPIDController.GetSetpoint();

    if (setpoint + ArmSpeed * direction < MaxArmAngle && setpoint + ArmSpeed * direction > MinArmAngle)//buffer could be applied here
    {
        ArmPIDController.SetSetpoint(setpoint + ArmSpeed);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }
}

void Subsystems::moveTelescopethingy(bool direction)
{
    auto setpoint = TelePIDController.GetSetpoint();

    if (setpoint + TeleSpeed * direction < MaxTeleAngle && setpoint +TeleSpeed * direction > MinTeleAngle)//buffer could be applied here
    {
       TelePIDController.SetSetpoint(setpoint +TeleSpeed);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }
}

void Subsystems::SetArmPIDTarget(int target)
{
    if (target < MaxArmAngle && target > MinArmAngle)
    {
        ArmPIDController.SetSetpoint(target);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }

}
void Subsystems::SetTelePIDTarget(int target)
{
    if (target < MaxTeleAngle && target > MinTeleAngle)
    {
        TelePIDController.SetSetpoint(target);
    }
    else
    {
        std::cout << "Tele out of Bounds" << std::endl;
    }
}


int Subsystems::GetArmEncoderAngle()
{
    return (ArmMotorEncoder.GetAbsolutePosition() - 0.16)*(90/0.065);
}