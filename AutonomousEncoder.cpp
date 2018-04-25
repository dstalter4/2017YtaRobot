////////////////////////////////////////////////////////////////////////////////
/// @file AutonomousEncoder.cpp
///
/// Implementation of autonmous encoder based routines.
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
// @method CmsdRobot::GetEncoderRotationsFromInches
///
/// Returns a number of encoder turns based on input distance in
/// inches and a diameter of the object turning.  It is based on
/// the 4x (4096) quadrature encoders.
///
////////////////////////////////////////////////////////////////
int CmsdRobot::GetEncoderRotationsFromInches(int inches, float diameter, bool bUseQuadEncoding)
{
    // c = PI*d
    // (PI*d)/4096 is ratio of one encoder turn to a distance of
    // travel of one diameter rotations.  To scale up, use cross
    // multiply and divide.  Therefore:
    //   PI * d     x(in.)
    //  -------- = --------
    //    4096      y(rot)
    // x and d are inputs, so solve for y.
    // y = (4096x)/(PI*d)
    // This is for quadrature encoding, so if analog (single)
    // encoding is desired, the result needs to be divided by four.
    // If 4" wheels are in use, 3911.39188 turns = 12"
    volatile int numerator = QUADRATURE_ENCODING_ROTATIONS * inches;
    volatile float denominator = M_PI * diameter;
    volatile int result = numerator / denominator;
    //int result = (QUADRATURE_ENCODING_ROTATIONS * inches) / (M_PI * diameter);
    
    if (!bUseQuadEncoding)
    {
        result /= 4;
    }
    
    return result;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousEncoderDrive
///
/// Autonomous method to drive the robot controlled by the
/// encoders.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousEncoderDrive(float speed, float distance, EncoderDirection direction)
{
    // 2017 LEFT FORWARD DRIVE IS NEGATIVE
    // 2017 RIGHT FORWARD DRIVE IS POSITIVE
    // 2017 LEFT ENCODER VALUE DECREASES GOING FORWARD
    // 2017 RIGHT ENCODER VALUE INCREASES GOING FORWARD
    
    // 1024 is the encoder output after one rotation
    // 2017 Wheels are 4", therefore circumference = 4*PI
    // Ratio is (4*PI)/1024 = PI/256
    //
    //   x  |  y  
    // -----------
    //  12" | (3072 / PI) = 977.84797
    //  24" | (6144 / PI) = 1955.6959
    //  ... | ...
    // 120" | (30720 / PI) = 9778.4797
    // 132" | (33792 / PI) = 10756.32767
    // 144" | (36864 / PI) = 11734.17564
    
    // New drive operation, tare encoders
    m_pLeftDriveMotor->TareEncoder();
    m_pRightDriveMotor->TareEncoder();
    
    // Start the safety timer        
    m_pSafetyTimer->Reset();
    m_pSafetyTimer->Start();
    
    // Get initial encoder values
    int leftEncVal = 0;
    int rightEncVal = 0;
    
    do
    {
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
        
        // Set speeds, adjust below if needed
        float leftDriveSpeed = speed;
        float rightDriveSpeed = speed;
        float leftDriveScale = 1.0F;
        float rightDriveScale = 1.0F;
        
        // Get encoder values to always be positive, based on direction.
        // Also scale the drive motors for direction.
        switch (direction)
        {
            case FORWARD:
            {
                leftEncVal = -(m_pLeftDriveMotor->GetEncoderValue());
                rightEncVal = m_pRightDriveMotor->GetEncoderValue();
                leftDriveScale = -1.0F;
                
                break;
            }
            case REVERSE:
            {
                leftEncVal = m_pLeftDriveMotor->GetEncoderValue();
                rightEncVal = -(m_pRightDriveMotor->GetEncoderValue());
                rightDriveScale = -1.0F;
                
                break;
            }
            default:
            {
                break;
            }
        }
    
        // If left is ahead of right, slow down left, increase right
        if (leftEncVal > rightEncVal)
        {
            leftDriveSpeed -= CmsdRobotAutonomous::DRIVE_COMPENSATE_SPEED;
            rightDriveSpeed += CmsdRobotAutonomous::DRIVE_COMPENSATE_SPEED;
        }
        // If right is head of left, slow down right, increase left
        else if (leftEncVal < rightEncVal)
        {
            leftDriveSpeed += CmsdRobotAutonomous::DRIVE_COMPENSATE_SPEED;
            rightDriveSpeed -= CmsdRobotAutonomous::DRIVE_COMPENSATE_SPEED;
        }
        else
        {
        }
        
        // Motors on
        m_pLeftDriveMotor->Set(leftDriveSpeed * leftDriveScale);
        m_pRightDriveMotor->Set(rightDriveSpeed * rightDriveScale);
        
        // Send stats back to the smart dashboard
        if (DEBUG_PRINTS)
        {
            SmartDashboard::PutNumber("Enc. L: ", leftEncVal);
            SmartDashboard::PutNumber("Enc. R: ", rightEncVal);
            SmartDashboard::PutNumber("Enc Diff: ", leftEncVal - rightEncVal);
        }
    
    } while ( (leftEncVal < GetEncoderRotationsFromInches(distance, DRIVE_WHEEL_DIAMETER_INCHES)) 
           && (rightEncVal < GetEncoderRotationsFromInches(distance, DRIVE_WHEEL_DIAMETER_INCHES))
           && (m_pSafetyTimer->Get() <= CmsdRobotAutonomous::ENCODER_DRIVE_MAX_DELAY_S) );
    
    // Motors back off    
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    AutonomousBackDrive(direction);
    
    // Stop and reset the safety timer
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
}
