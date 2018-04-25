////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobotAutonomous2.cpp
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
// @method CmsdRobot::AutonomousRoutine2
///
/// Autonomous routine 2.  This routine places a gear on one of
/// the angled lift pegs.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousRoutine2(bool bBoilerSide)
{
    float moveFromWallDistance = 0.0F;
    float moveToPegDistance = 0.0F;
    
    if (bBoilerSide)
    {
        moveFromWallDistance = CmsdRobotAutonomous::ENCODER_BOILER_GEAR_FIRST_IN;
        moveToPegDistance = CmsdRobotAutonomous::ENCODER_BOILER_GEAR_SECOND_IN;
    }
    else
    {
        moveFromWallDistance = CmsdRobotAutonomous::ENCODER_NON_BOILER_GEAR_FIRST_IN;
        moveToPegDistance = CmsdRobotAutonomous::ENCODER_NON_BOILER_GEAR_SECOND_IN;
    }
    
    // Drive forward
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, moveFromWallDistance/*CmsdRobotAutonomous::ENCODER_BOILER_GEAR_FIRST_IN*/, FORWARD);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    
    // If red, turn left
    if (m_AllianceColor == Alliance::kRed)
    {
        if (bBoilerSide)
        {
            AutonomousGyroLeftTurn(CmsdRobotAutonomous::TURN_TO_GEAR_FROM_SIDES_RED, CmsdRobotAutonomous::TURN_SPEED);
        }
        else
        {
            AutonomousGyroRightTurn(CmsdRobotAutonomous::TURN_TO_GEAR_FROM_SIDES_RED, CmsdRobotAutonomous::TURN_SPEED);
        }
    }
    // If blue, turn right
    else if (m_AllianceColor == Alliance::kBlue)
    {
        if (bBoilerSide)
        {
            AutonomousGyroRightTurn(CmsdRobotAutonomous::TURN_TO_GEAR_FROM_SIDES_BLUE, CmsdRobotAutonomous::TURN_SPEED);
        }
        else
        {
            AutonomousGyroLeftTurn(CmsdRobotAutonomous::TURN_TO_GEAR_FROM_SIDES_BLUE, CmsdRobotAutonomous::TURN_SPEED);
        }
    }
    else
    {
    }
    
    // Drive forward, place the gear, wait for the human player to grab it
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, moveToPegDistance/*CmsdRobotAutonomous::ENCODER_BOILER_GEAR_SECOND_IN*/, FORWARD);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_LONG_S * 2.0F);
    
    // Stop here for now
    return;
    
    // Back away from the airship
    AutonomousEncoderDrive(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, CmsdRobotAutonomous::ENCODER_DRIVE_GEAR_REVERSE_IN, REVERSE);
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    
        // If red, turn right
    if (m_AllianceColor == Alliance::kRed)
    {
        AutonomousGyroRightTurn(CmsdRobotAutonomous::TURN_TO_BOILER_NEAR_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    // If blue, turn left
    else if (m_AllianceColor == Alliance::kBlue)
    {
        AutonomousGyroLeftTurn(CmsdRobotAutonomous::TURN_TO_BOILER_NEAR_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    else
    {
    }
    
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
