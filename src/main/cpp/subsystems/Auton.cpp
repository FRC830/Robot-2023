#include "../include/subsystems/Auton.h"
#include <iostream>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Subsystems.h"

void Auton::runAuton (int mode, DriveSubsystem& m_drive, Subsystems& m_subsystems, int counter)
{
    std::cout << "run auton" << std::endl;
    switch (mode)
    {
        case 0:
            taxi(m_drive, counter);
            break;
        case 1: 
            DockingRight(m_drive, counter);
        case 2:
            DockingLeft(m_drive, counter);
        case 3:
            ScorePieceTop(m_subsystems, m_drive, counter);
        default:
            taxi(m_drive, counter);
            break;
    }
}

void Auton::taxi(DriveSubsystem& m_drive, int counter)
{
    if (counter < 275){
        m_drive.Drive(0.5_mps, 0_mps, 0_deg_per_s, false);
    }
}

void Auton::taxiWithLowScore(DriveSubsystem& m_drive, int counter)
{
    if (counter < 25)
    {
        m_drive.Drive(6_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 300)
    {
        m_drive.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
    }
}

void Auton::DockingRight(DriveSubsystem& m_driveSubsystem, int counter)
{
    if (counter < 25)
    {
        m_driveSubsystem.Drive(6_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 275)
    {
        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 325)
    {
        m_driveSubsystem.Drive(0_mps, 0_mps, -90_deg_per_s, false);
    } else if (counter < 375)
    {
        m_driveSubsystem.Drive(1_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 425)
    {
        m_driveSubsystem.Drive(0_mps, 0_mps, -90_deg_per_s, false);
    } else if (counter < 475)
    {
        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
        BalanceOnStation(m_driveSubsystem);
    }
}

void Auton::DockingLeft(DriveSubsystem& m_driveSubsystem, int counter)
{
    if (counter < 275)
    {
        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 325)
    {
        m_driveSubsystem.Drive(0_mps, 0_mps, 90_deg_per_s, false);
    } else if (counter < 375)
    {
        m_driveSubsystem.Drive(1_mps, 0_mps, 0_deg_per_s, false);
    } else if (counter < 425)
    {
        m_driveSubsystem.Drive(0_mps, 0_mps, 90_deg_per_s, false);
    } else if (counter < 475)
    {
        m_driveSubsystem.Drive(1.5_mps, 0_mps, 0_deg_per_s, false);
        BalanceOnStation(m_driveSubsystem);
    }
}


void Auton::ScorePieceTop(Subsystems& m_subsystems, DriveSubsystem& m_drive, int counter) {

    

}


//util stuff

double Auton::getPitchForBalance()
{
    return std::atan2((- mAccel.GetX()) , std::sqrt(mAccel.GetY() * mAccel.GetY() + mAccel.GetZ() * mAccel.GetZ())) * 57.3; // pitch angle
}

double Auton::getRollForBalance()
{
    return std::atan2(mAccel.GetY() , mAccel.GetZ()) * 57.3; // roll angle
}

void Auton::BalanceOnStation(DriveSubsystem& m_drive)
{
    double pitchAngle = getPitchForBalance();
    double rollAngle = getRollForBalance();

    if (pitchAngle < -deadzoneAngle)
    {
        m_drive.Drive((units::velocity::meters_per_second_t)balanceSpeed, 0_mps, 0_deg_per_s, false);
    } else if (pitchAngle > deadzoneAngle)
    {
        m_drive.Drive(-(units::velocity::meters_per_second_t)balanceSpeed, 0_mps, 0_deg_per_s, false);
    }
}