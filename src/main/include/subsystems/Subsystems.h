#include <frc/Solenoid.h>
#include <frc/controller/PIDController.h>
#include <rev/CANSparkMax.h>
#include <frc2/command/SubsystemBase.h>
#include <frc/AnalogEncoder.h>
#include "../Constants.h"


class Subsystems : public frc2::SubsystemBase
{
    public:
        Subsystems();

        Subsystems(const Subsystems &) = delete;
        Subsystems &operator=(const Subsystems &) = delete;


        void SetGrabberWheels(bool direction);
        void ToggleGrabberPnumatics ();
        void RotateArm(bool direction);
        void moveTelescopethingy(bool direction);
        void StopTelescope();
        
        void DisableGrabberWheels();
        void SetArmPIDF (double p, double i, double d);
        void SetArmPIDTarget (int target);
        void SetTelePIDF (double p, double i, double d);
        void SetTelePIDTarget (int target);

        void SubsystemsInit ();
        void SubsystemsPeriodic ();
    private:
        //Constants
        double GrabberWheelSpeeds = 1.0;

        double ArmSpeed = 0.75;
        int MinArmAngle = -60;
        int MaxArmAngle = 62;
        int ArmAngleBufferSize = 20;

        double TeleSpeed = 0.30;
        int MinTeleAngle = -55000000;
        int MaxTeleAngle = 10000000;
        int TeleAngleBufferSize = 2;   
        
        //Status Variables
        bool GrabberOnOff = false;
 
        //PID Control
        frc2::PIDController ArmPIDController;
        frc2::PIDController TelePIDController;

        //Physical objects
        frc::Solenoid GrabberSolenoid{frc::PneumaticsModuleType::CTREPCM, 0};

        rev::CANSparkMax GrabberWheelsMotor =  rev::CANSparkMax(10, rev::CANSparkMaxLowLevel::MotorType::kBrushless);

        rev::CANSparkMax ArmMotor = rev::CANSparkMax(11, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        frc::AnalogEncoder ArmMotorEncoder {5};


        rev::CANSparkMax teleMotor = rev::CANSparkMax(12, rev::CANSparkMaxLowLevel::MotorType::kBrushless);
        rev::SparkMaxRelativeEncoder TeleMotorEncoder = teleMotor.GetEncoder();

        

  int GetArmEncoderAngle();
};