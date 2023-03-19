#include "DriveSubsystem.h"
#include "Subsystems.h"
#include <frc/BuiltInAccelerometer.h>
#include <cmath>

class Auton 
{
    public: 
        void runAuton (int mode, DriveSubsystem& m_drive, Subsystems& m_subsystems, int counter);
        double deadzoneAngle {10};
        double balanceSpeed {1};

        frc::BuiltInAccelerometer mAccel{};
    
    private:
        double getPitchForBalance();
        double getRollForBalance();

        void taxi(DriveSubsystem& m_drive, int counter);
        void taxiWithLowScore(DriveSubsystem& m_drive, int counter);
        void DockingRight(DriveSubsystem& m_drive, int counter);
        void DockingLeft(DriveSubsystem& m_drive, int counter);
        void BalanceOnStation(DriveSubsystem& m_drive);
        void ScorePieceTop(Subsystems& m_subsystems, DriveSubsystem& m_drive, int counter);
};