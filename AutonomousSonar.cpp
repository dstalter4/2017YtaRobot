////////////////////////////////////////////////////////////////////////////////
/// @file AutonomousSonar.cpp
///
/// Implementation of autonmous sonar routines.
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
// @method CmsdRobot::AutonomousSonarDrive
///
/// Autonomous method to drive the robot controlled by the
/// sonar sensors.
///
////////////////////////////////////////////////////////////////
bool CmsdRobot::AutonomousSonarDrive(SonarDriveDirection driveDirection, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist)
{    
    // Set directions based on drive state
    uint32_t sideDirection = driveState & SONAR_DRIVE_STATE_SIDE_MASK;
    uint32_t lateralDirection = driveState & SONAR_DRIVE_STATE_LATERAL_MASK;
    
    uint32_t frontGuideSensor = 0U;
    uint32_t backGuideSensor = 0U;
    uint32_t destGuideSensorA = 0U;
    uint32_t destGuideSensorB = 0U;
    
    // Set values based on which side is guiding drive        
    switch (lateralDirection)
    {
        case FORWARD_GUIDE:
        {
            destGuideSensorA = m_I2cData.m_FrontSonarB;
            destGuideSensorB = m_I2cData.m_FrontSonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_LeftSonarB;
                    backGuideSensor = m_I2cData.m_LeftSonarA;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_RightSonarB;
                    backGuideSensor = m_I2cData.m_RightSonarB;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        case REVERSE_GUIDE:
        {
            destGuideSensorA = m_I2cData.m_BackSonarA;
            destGuideSensorB = m_I2cData.m_BackSonarB;
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_LeftSonarA;
                    backGuideSensor = m_I2cData.m_LeftSonarB;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    frontGuideSensor = m_I2cData.m_RightSonarB;
                    backGuideSensor = m_I2cData.m_RightSonarB;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start with defaults of off and no turning
    float leftDriveSpeed = OFF;
    float rightDriveSpeed = OFF;    
    bool bLeftTurn = false;
    bool bRightTurn = false;
    bool bCanOverrideTurn = true;
    
    // Make sure we're close enough to a guiding structure
    if (    (frontGuideSensor < CmsdRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES)
         && (backGuideSensor < CmsdRobotAutonomous::SONAR_MIN_DRIVE_ENABLE_INCHES) )
    {
        // Start assuming a straight drive
        leftDriveSpeed = CmsdRobotAutonomous::SONAR_DRIVE_LEFT_SPEED;
        rightDriveSpeed = CmsdRobotAutonomous::SONAR_DRIVE_RIGHT_SPEED;
        
        // Check for turning need.  The first checks here determine
        // if we need to turn the robot left or right, and are to
        // align the robot at a (mostly) right angle.
        if (frontGuideSensor > backGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((frontGuideSensor - backGuideSensor) > CmsdRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else if (backGuideSensor > frontGuideSensor)
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
            
            // If the robot is offset too sharply, don't allow
            // the guiding below to override what we want to do.
            if ((backGuideSensor - frontGuideSensor) > CmsdRobotAutonomous::SONAR_MAX_ALLOWED_READING_DIFF)
            {
                bCanOverrideTurn = false;
            }
        }
        else
        {
        }
        
        // Align with the destination distance.  These checks, unlike the ones
        // above, are to move towards the target distance from the wall.
        if (bCanOverrideTurn && (frontGuideSensor > destSideDist))
        {
            switch (sideDirection)
            {
                case LEFT_GUIDE:
                {
                    bLeftTurn = true;
                    break;
                }
                case RIGHT_GUIDE:
                {
                    bRightTurn = true;
                    break;
                }
                default:
                {
                    break;
                }
            }
        }
        
        // Set the motor speed values
        if (bLeftTurn)
        {
            leftDriveSpeed -= CmsdRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed += CmsdRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else if (bRightTurn)
        {
            leftDriveSpeed += CmsdRobotAutonomous::SONAR_COMPENSATE_LEFT_SPEED;
            rightDriveSpeed -= CmsdRobotAutonomous::SONAR_COMPENSATE_RIGHT_SPEED;
        }
        else
        {
        }
        
        // Speeds are now set based on need to turn.  Enable motors
        // only if we have not reached the maximum distance.
        if ((destGuideSensorA < destLateralDist) && (destGuideSensorB < destLateralDist))
        {
            if (driveDirection == SONAR_FORWARD)
            {
                m_pLeftDriveMotor->Set(leftDriveSpeed);
                m_pRightDriveMotor->Set(rightDriveSpeed);
            }
            else if (driveDirection == SONAR_REVERSE)
            {
                m_pLeftDriveMotor->Set(-leftDriveSpeed);
                m_pRightDriveMotor->Set(-rightDriveSpeed);
            }
            else
            {
            }
            
            return false;
        }
    }
    
    return true;
}
