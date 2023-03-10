#include "../include/subsystems/Auton.h"
#include <iostream>

void Auton::runAuton (int mode)
{
    
    std::cout << "run auton" << std::endl;
    switch (mode)
    {
    case 0:
        taxi();
        break;
    
    default:
        break;
    }
}

void Auton::taxi()
{
    std::cout << "taxi auton" << std::endl;
    // m_drive.Drive(2.133_mps, 0_mps, 0_deg_per_s, false);
    m_drive->Drive(-1.05_mps, 0_mps, 0_deg_per_s, false);
}