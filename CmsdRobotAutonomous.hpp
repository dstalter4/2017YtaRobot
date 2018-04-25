////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobot.hpp
///
/// This is the class declaration for a FRC robot derived from the WPI library
/// base classes.  The SampleRobot class is the base of a robot application that
/// will automatically call the Autonomous and OperatorControl methods at the
/// right time as controlled by the switches on the driver station or the field
/// controls.
///
/// CMSD FRC 2017
/// Author(s): David Stalter
/// @Edit History
/// - dts   12-MAR-2017 Created.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef CMSDROBOTAUTONOMOUS_HPP
#define CMSDROBOTAUTONOMOUS_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "WPILib.h"                 // For FRC library

// C++ INCLUDES
#include "CmsdRobot.hpp"            // For inline autonomous function declarations

////////////////////////////////////////////////////////////////
// @class CmsdRobot
///
/// Derived class from SampleRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
namespace CmsdRobotAutonomous
{
//public:
    // TYPEDEFS
    // (none)
    
    // ENUMS
    // (none)    
    
    // STRUCTS
    // (none)
    
    // MEMBER VARIABLES
    
    // CONSTS
    
    // Autonomous Mode Constants
    // Note: Only enable one autonomous routine!
    // Note: Autonomous routines currently NOT controlled by
    // physical switches on the robot.
    static const bool       ROUTINE_1                           = true;
    static const bool       ROUTINE_2                           = false;
    static const bool       ROUTINE_3                           = false;
    static const bool       TEST_ENABLED                        = false;

    // Autonomous drive speed constants
    static constexpr float  DRIVE_SPEED_SLOW                    =  0.30F;//0.35F;
    static constexpr float  DRIVE_SPEED_FAST                    =  0.50F;                    
    static constexpr float  DRIVE_COMPENSATE_SPEED              =  0.02F;
    static constexpr float  DRIVE_RAMMING_SPEED                 =  0.60F;
    static constexpr float  TURN_SPEED                          =  0.50F;//0.35F;
    static constexpr float  COUNTERACT_COAST_MOTOR_SPEED        =  0.20F;
    
    // Autonomous angle constants
    static constexpr float  TURN_ANGLE_SLOP_DEGREES             = -7.50F;
    static constexpr float  TURN_ANGLE_EXTRA_SLOP_DEGREES       = -3.00F;
    static constexpr float  TURN_TO_GEAR_FROM_SIDES_BLUE        = 46.00F;
    static constexpr float  TURN_TO_GEAR_FROM_SIDES_RED         = 48.50F;
    static constexpr float  TURN_TO_BOILER_MIDDLE_ANGLE_DEGREES = 97.50F;
    static constexpr float  TURN_TO_BOILER_NEAR_ANGLE_DEGREES   = 190.0F;
    static constexpr float  TURN_TO_BOILER_LOAD_ANGLE_DEGREES   = 30.00F;
    static constexpr float  FORTY_FIVE_DEGREE_TURN_ANGLE        = 45.00F;
    static constexpr float  NINETY_DEGREE_TURN_ANGLE            = 90.00F;
    static constexpr float  ONE_EIGHTY_DEGREE_TURN_ANGLE        = 180.0F;
    
    // Autonomous delay constants
    static const int        SHOOT_DELAY_S                       =      8;
    static const int        GATHER_FUEL_BALLS_DELAY_S           =      2;
    static constexpr float  COUNTERACT_COAST_TIME_S             =  0.25F;
    static constexpr float  ENCODER_DRIVE_MAX_DELAY_S           =  5.00F;
    static constexpr float  DELAY_SHORT_S                       =  0.50F;
    static constexpr float  DELAY_MEDIUM_S                      =  1.00F;
    static constexpr float  DELAY_LONG_S                        =  2.00F;
    
    // Autonomous misc. motors constants
    static constexpr float  FUEL_PUMP_MOTOR_SPEED               =  0.50F;
    static constexpr float  FUEL_SHOOT_MOTOR_SPEED              =  0.75F;
    static constexpr float  FUEL_INJECT_MOTOR_SPEED             =  0.60F;
    
    // Autonomous encoder drive constants
    static const int        ENCODER_DRIVE_STRAIGHT_IN           =   12*8;
    static const int        ENCODER_DRIVE_GEAR_FORWARD_IN       =   12*8;
    static const int        ENCODER_DRIVE_GEAR_REVERSE_IN       =   12*3;
    static const int        ENCODER_DRIVE_GEAR_TO_BOILER_IN     =   12*4;
    static const int        ENCODER_BOILER_GEAR_FIRST_IN        =   66;//37;
    static const int        ENCODER_BOILER_GEAR_SECOND_IN       =   144;//86; deliberately high, operation will time out
    static const int        ENCODER_NON_BOILER_GEAR_FIRST_IN    =   78;//45;
    static const int        ENCODER_NON_BOILER_GEAR_SECOND_IN   =   144;//75; deliberately high, operation will time out
    
    static const int        ENCODER_DRIVE_FIRST_DIST_IN         =   12*7;
    static const int        ENCODER_DRIVE_SECOND_DIST_IN        =   12*3;
    static const int        ENCODER_DRIVE_THIRD_DIST_IN         =   12*1;
    static constexpr float  ENCODER_DRIVE_FIRST_DIST_TURNS      = 28000.0F;
    static constexpr float  ENCODER_DRIVE_SECOND_DIST_TURNS     = 15000.0F;
    static constexpr float  ENCODER_DRIVE_THIRD_DIST_TURNS      =  4000.0F;
    
    // Autonomous sonar drive constants
    static const int        SONAR_LATERAL_DRIVE_DIST_INCHES     =   7*12;
    static const int        SONAR_SIDE_DRIVE_DIST_INCHES        =      6;
    static const int        SONAR_MIN_DRIVE_ENABLE_INCHES       =  10*12;
    static const int        SONAR_INIT_TURN_DIST_INCHES         =      5;
    static const int        SONAR_MAX_ALLOWED_READING_DIFF      =      2;
    static const unsigned   SONAR_BUMPER_CLEARANCE_DIST_INCHES  =      4;
    static constexpr float  SONAR_ROUTINE_TIME_S                =  5.00F;
    static constexpr float  SONAR_DRIVE_LEFT_SPEED              = -0.10F;
    static constexpr float  SONAR_DRIVE_RIGHT_SPEED             =  0.10F;
    static constexpr float  SONAR_COMPENSATE_LEFT_SPEED         = -0.05F;
    static constexpr float  SONAR_COMPENSATE_RIGHT_SPEED        =  0.05F;
    
} // End namespace



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousDelay
///
/// Waits for a specified amount of time in autonomous.  Used
/// while an operation is ongoing but not yet complete, and
/// nothing else needs to occur.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousDelay(float time)
{
    m_pAutonomousTimer->Start();
    while (m_pAutonomousTimer->Get() < time) {}
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousDriveSequence
///
/// Drives during autonomous for a specified amount of time.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousDriveSequence(float speed, float time)
{
    // First turn the motors on
    m_pLeftDriveMotor->Set(-speed);
    m_pRightDriveMotor->Set(speed);

    // Time it
    AutonomousDelay(time);

    // Motors back off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousBackDrive
///
/// Back drives the motors to abruptly stop the robot.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousBackDrive(EncoderDirection currentRobotDirection)
{
    float leftSpeed = CmsdRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    float rightSpeed = CmsdRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    switch (currentRobotDirection)
    {
        // If we are currently going forward, right motor back drive is negative
        case FORWARD:
        {
            rightSpeed *= -1.0F;
            break;
        }
        // If we are currently going backward, left motor back drive is negative
        case REVERSE:
        {
            leftSpeed *= -1.0F;
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Counteract coast
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(CmsdRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    m_pSafetyTimer->Reset();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousBackDriveTurn
///
/// Back drives the motors to abruptly stop the robot during
/// a turn.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::AutonomousBackDriveTurn(GyroDirection currentGyroDirection)
{
    float leftSpeed = CmsdRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    float rightSpeed = CmsdRobotAutonomous::COUNTERACT_COAST_MOTOR_SPEED;
    
    // Left turns have +/+ to the motors
    // Right turns have -/- to the motors
    
    // If the turn is left, counteract is -/-
    if (currentGyroDirection == LEFT_TURN)
    {
        leftSpeed *= -1.0F;
        rightSpeed *= -1.0F;
    }
    
    // Counteract coast
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    // Delay
    AutonomousDelay(CmsdRobotAutonomous::COUNTERACT_COAST_TIME_S);
    
    // Motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    m_pSafetyTimer->Reset();
}

#endif // CMSDROBOTAUTONOMOUS_HPP
