////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobotAutonomous3.cpp
///
/// Implementation of autonmous routines for CmsdRobot.
///
/// CMSD FRC 2017
/// Author: David Stalter
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"                // Robot class declaration
#include "CmsdRobotAutonomous.hpp"      // Autonomous declarations



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousRoutine3
///
/// Autonomous routine 3.  This assumes starting by the boiler
/// and shoots.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousRoutine3()
{
    /*
    if (m_AllianceColor == Alliance::kRed)
    {
    }
    else if (m_AllianceColor == Alliance::kBlue)
    {
    }
    else
    {
    }
    */
    
    // First angle the shooter backwards (towards frame of robot)
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    m_pFuelShooterAngleMotor->Set(SHOOTER_ANGLE_SCALING);
    while (m_pShooterAngleDownLimitSwitch->Get() && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
    {
    }
    m_pFuelShooterAngleMotor->Set(OFF);
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    AutonomousShoot();
    
    // Stop here for now
    //return;
    
    if (m_AllianceColor == Alliance::kRed)
    {
        //AutonomousGyroRightTurn(CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE, CmsdRobotAutonomous::TURN_SPEED);
        
        // Veer to the right
        m_pLeftDriveMotor->Set(-(CmsdRobotAutonomous::DRIVE_SPEED_SLOW + 0.10F));
        m_pRightDriveMotor->Set(CmsdRobotAutonomous::DRIVE_SPEED_SLOW);// - 0.05F);
        m_pSafetyTimer->Start();
        while (m_pSafetyTimer->Get() < 3.0F)
        {
        }
        m_pLeftDriveMotor->Set(OFF);
        m_pRightDriveMotor->Set(OFF);
        m_pSafetyTimer->Stop();
        m_pSafetyTimer->Reset();
    }
    else if (m_AllianceColor == Alliance::kBlue)
    {
        AutonomousDriveSequence(CmsdRobotAutonomous::DRIVE_SPEED_FAST, 1.0F);
        AutonomousGyroLeftTurn(CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE, CmsdRobotAutonomous::TURN_SPEED);
        
        // Veer to the left
        m_pLeftDriveMotor->Set(-(CmsdRobotAutonomous::DRIVE_SPEED_SLOW));// - 0.05F));
        m_pRightDriveMotor->Set(CmsdRobotAutonomous::DRIVE_SPEED_SLOW);// + 0.20F);
        m_pSafetyTimer->Start();
        while (m_pSafetyTimer->Get() < 3.0F)
        {
        }
        m_pLeftDriveMotor->Set(OFF);
        m_pRightDriveMotor->Set(OFF);
        m_pSafetyTimer->Stop();
        m_pSafetyTimer->Reset();
    }
    else
    {
    }
}
