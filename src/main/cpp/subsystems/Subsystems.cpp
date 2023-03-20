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

  

TelePIDController.SetSetpoint(TeleMotorEncoder.GetPosition());
ArmPIDController.SetSetpoint(GetArmEncoderAngle());

  frc::SmartDashboard::PutBoolean("Set this to True if Set Tele PID Target", false);
  frc::SmartDashboard::PutBoolean("Set this to True if Set Arm PID Target", false);

  frc::SmartDashboard::PutNumber("Arm PID target to Set to for manual setting", 0);
  frc::SmartDashboard::PutNumber("Tele PID target to Set to for manual setting", 0);

  hasCalibratedTele = false;
    
}

void Subsystems::SubsystemsPeriodic()
{
    frc::SmartDashboard::PutNumber("Arm Motor Speed From PID", ArmPIDController.Calculate(GetArmEncoderAngle()));
    ArmMotor.Set(ArmPIDController.Calculate(GetArmEncoderAngle()));
    SetArmPIDF
    (
        frc::SmartDashboard::GetNumber("ArmPCoefficient", pidf::kArmP), 
        frc::SmartDashboard::GetNumber("ArmI", pidf::kArmI), 
        frc::SmartDashboard::GetNumber("ArmD", pidf::kArmD)
    ); //-15, full extension is scoring
    // SetArmPIDF
    // (
    //     sin(GetArmEncoderAngle()*(3.14 /180)) * TeleMotorEncoder.GetPosition() * 
    //         frc::SmartDashboard::GetNumber("ArmPCoefficient", pidf::kArmP) +
    //         frc::SmartDashboard::GetNumber("ArmPAdder", 0), 
    //     frc::SmartDashboard::GetNumber("ArmI", pidf::kArmI), 
    //     frc::SmartDashboard::GetNumber("ArmD", pidf::kArmD)
    // );

    
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

    if (!hasCalibratedTele)
    {
        if (!teleSwitch.Get())
        {
            hasCalibratedTele = true;
            TeleMotorEncoder.SetPosition(0);
            teleMotor.Set(0);
            TelePIDController.SetSetpoint(-1);
        }
        else
        {
            teleMotor.Set(0.175);
        }

    }
    else 
    {
        teleMotor.Set(TelePIDController.Calculate(TeleMotorEncoder.GetPosition()));
    }

    frc::SmartDashboard::PutBoolean("limit switch", teleSwitch.Get());
}

void Subsystems::SetGrabberWheels(bool direction)
{
    if (direction)
    {GrabberWheelsMotor.Set(GrabberWheelSpeeds *-1);}
    else 
    {
        GrabberWheelsMotor.Set(GrabberWheelSpeeds);
    }
}

void Subsystems::DisableGrabberWheels()
{
    GrabberWheelsMotor.Set(0);
}
void Subsystems::ToggleGrabberPnumatics()
{
    
    if (!GrabberOnOff)
    {
        GrabberSolenoid.Set(true);
        GrabberOnOff = false;
    }
    else
    {
        GrabberSolenoid.Set(false);
        GrabberOnOff = true;
    }

    GrabberOnOff = !GrabberOnOff;

}
void Subsystems::SetGrabberPnumatics(bool state)
{
    GrabberSolenoid.Set(state);
    GrabberOnOff = state;
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

void Subsystems::RotateArm(bool input)
{

    int direction;
    input ? direction = 1 : direction = -1;

    auto setpoint = ArmPIDController.GetSetpoint();

    if (setpoint + ArmSpeed * direction < MaxArmAngle && setpoint + ArmSpeed * direction > MinArmAngle)//buffer could be applied here
    {
        ArmPIDController.SetSetpoint(setpoint + ArmSpeed * direction);
    }
    else
    {
        std::cout << "Arm out of Bounds" << std::endl;
    }
}

void Subsystems::moveTelescopethingy(bool input)
{
    int direction;
    input ? direction = 1 : direction = -1;
    auto setpoint = TelePIDController.GetSetpoint();

    if (setpoint + TeleSpeed * direction < MaxTeleAngle && setpoint + TeleSpeed * direction > MinTeleAngle)//buffer could be applied here
    {
        std::cout << "moving tele" << std::endl;   
       TelePIDController.SetSetpoint(setpoint + TeleSpeed * direction);
    }
    else
    {
        std::cout << "Tele out of Bounds" << std::endl;
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
    if (target > MaxTeleAngle && target < MinTeleAngle)
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
    