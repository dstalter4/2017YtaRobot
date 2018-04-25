////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobotAutonomous1.cpp
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
// @method CmsdRobot::AutonomousRoutine1
///
/// Autonomous routine 1.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousRoutine1()
{
    m_pLeftDriveMotor->TareEncoder();
    m_pRightDriveMotor->TareEncoder();
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    
    // Drive forward, place the gear, wait for the human player to grab it
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, CmsdRobotAutonomous::ENCODER_DRIVE_GEAR_FORWARD_IN, FORWARD);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_LONG_S);
    
    // Back away from the airship
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, CmsdRobotAutonomous::ENCODER_DRIVE_GEAR_REVERSE_IN, REVERSE);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    
    // If red, turn right
    if (m_AllianceColor == Alliance::kRed)
    {
        AutonomousGyroRightTurn(CmsdRobotAutonomous::TURN_TO_BOILER_MIDDLE_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    // If blue, turn left
    else if (m_AllianceColor == Alliance::kBlue)
    {
        AutonomousGyroLeftTurn(CmsdRobotAutonomous::TURN_TO_BOILER_MIDDLE_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    else
    {
    }
    
    // Quick delay and drive forward
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, CmsdRobotAutonomous::ENCODER_DRIVE_GEAR_TO_BOILER_IN, FORWARD);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    
    // Angle the shooter
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    m_pFuelShooterAngleMotor->Set(SHOOTER_ANGLE_SCALING);
    while (m_pShooterAngleDownLimitSwitch->Get() && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
    {
    }
    m_pFuelShooterAngleMotor->Set(OFF);
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Take the shot
    AutonomousShoot();
}
