////////////////////////////////////////////////////////////////////////////////
/// @file AutonomousGyro.cpp
///
/// Implementation of autonmous gyroscope routines.
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
// @method CmsdRobot::AutonomousGyroLeftTurn
///
/// Turns the robot left based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool CmsdRobot::AutonomousGyroLeftTurn(float destAngle, float turnSpeed)
{
    // 2017 LEFT FORWARD DRIVE IS NEGATIVE
    // 2017 RIGHT FORWARD DRIVE IS POSITIVE    
    // 2017 LEFT TURNS DECREASE GYRO ANGLE
    
    float startAngle = m_pGyro->GetAngle();
    
    // Left turns are right motors forward, left motors reverse
    m_pLeftDriveMotor->Set(turnSpeed);
    m_pRightDriveMotor->Set(turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be decreasing
    while ((m_pGyro->GetAngle() > (startAngle - destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro: ", m_pGyro->GetAngle());
    }
    
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(LEFT_TURN);
    
    return true;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousGyroRightTurn
///
/// Turns the robot right based on gyro readings.
///
////////////////////////////////////////////////////////////////
bool CmsdRobot::AutonomousGyroRightTurn(float destAngle, float turnSpeed)
{
    // 2017 LEFT FORWARD DRIVE IS NEGATIVE
    // 2017 RIGHT FORWARD DRIVE IS POSITIVE
    // 2017 RIGHT TURNS INCREASE GYRO ANGLE
    
    float startAngle = m_pGyro->GetAngle();
    
    // Right turns are left motors forward, right motors reverse
    m_pLeftDriveMotor->Set(-turnSpeed);
    m_pRightDriveMotor->Set(-turnSpeed);
    
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Angle will be increasing
    while ((m_pGyro->GetAngle() < (startAngle + destAngle)) && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE))
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        SmartDashboard::PutNumber("Gyro: ", m_pGyro->GetAngle());
    }
    
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    m_pSafetyTimer->Stop();
    if (m_pSafetyTimer->Get() > SAFETY_TIMER_MAX_VALUE)
    {
        m_pSafetyTimer->Reset();
        return false;
    }
    m_pSafetyTimer->Reset();
    
    // Counteract coast
    AutonomousBackDriveTurn(RIGHT_TURN);
    
    return true;
}
