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
/// - dts   09-JAN-2016 Created from 2015.
/// - dts   08-JAN-2017 Ported from 2016.
///
////////////////////////////////////////////////////////////////////////////////

#ifndef CMSDROBOT_HPP
#define CMSDROBOT_HPP

// SYSTEM INCLUDES
#include <cmath>                        // For M_PI

// C INCLUDES
#include "WPILib.h"                     // For FRC library
#include "CameraServer.h"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>

// C++ INCLUDES
#include "TalonMotorGroup.hpp"          // For Talon group motor control

////////////////////////////////////////////////////////////////
// @class CmsdRobot
///
/// Derived class from SampleRobot.  The object that will
/// control all robot functionality.
///
////////////////////////////////////////////////////////////////
class CmsdRobot : public SampleRobot
{
public:

    // MEMBER FUNCTIONS
    
    // Autonomous routine
    void Autonomous();
    
    // Main operator drive control routine
    void OperatorControl();
    
    // Test mode routine
    void Test();
    
    // Constructor, destructor
    CmsdRobot();
    virtual ~CmsdRobot() {}
    
    // Do not declare or implement copy constructor or assignment
    // operator!  The base classes won't like it and most likely
    // it will crash the thread when running!
      
private:

    // TYPEDEFS
    typedef DriverStation::Alliance Alliance;
    typedef Relay::Value RelayValue;
    typedef DoubleSolenoid::Value SolenoidState;
    typedef TalonMotorGroup::FeedbackDevice FeedbackDevice;
    typedef TalonMotorGroup::ControlMode ControlMode;
    
    // ENUMS
    enum EncoderDirection
    {
        FORWARD,
        REVERSE,
        LEFT,
        RIGHT
    };
    
    enum GyroDirection
    {
        LEFT_TURN,
        RIGHT_TURN
    };
    
    enum SonarDriveState
    {
        NONE            = 0x00,
        LEFT_GUIDE      = 0x01,
        RIGHT_GUIDE     = 0x02,
        FORWARD_GUIDE   = 0x10,
        REVERSE_GUIDE   = 0x20
    };
    
    enum SonarDriveDirection
    {
        STOPPED,
        SONAR_FORWARD,
        SONAR_REVERSE
    };
    
    enum PulseState
    {
        MANUAL_PULSE,
        PULSE_IN_PROGRESS,
        PULSE_FINISHED_DELAY
    };
    
    enum AgitatorState
    {
        READY,
        AGITATING,
        REFRACTORY
    };
    
    enum GearState
    {
        MANUAL_INTAKE,
        AUTO_INTAKE,
        SETTLING
    };
    
    // STRUCTS
    struct TriggerChangeValues
    {
        bool bCurrentValue;
        bool bOldValue;
    };
    
    struct I2cData
    {
        static const uint32_t SONAR_FRONT_A_INDEX   = 1U;
        static const uint32_t SONAR_FRONT_B_INDEX   = 2U;
        static const uint32_t SONAR_LEFT_A_INDEX    = 3U;
        static const uint32_t SONAR_LEFT_B_INDEX    = 4U;
        static const uint32_t SONAR_BACK_A_INDEX    = 5U;
        static const uint32_t SONAR_BACK_B_INDEX    = 6U;
        static const uint32_t SONAR_RIGHT_A_INDEX   = 7U;
        static const uint32_t SONAR_RIGHT_B_INDEX   = 8U;
        static const uint32_t NUM_SONAR_SENSORS     = 8U;
        
        // Sensor locations are relative to robot facing forward.
        // This structure must match the Rioduino code.
        //
        //       Front Sonars
        //       A          B
        //   |--\_/--    --\_/--|
        //   |                  |
        // L > B              A < R
        // e |                  | i
        // f |                  | g
        // t > A              B < h
        //   |                  | t
        //   |--/-\--------/-\--|
        //       B          A
        //       Back  Sonars
        
        uint8_t m_Header;
        uint8_t m_FrontSonarA;
        uint8_t m_FrontSonarB;
        uint8_t m_LeftSonarA;
        uint8_t m_LeftSonarB;
        uint8_t m_BackSonarA;
        uint8_t m_BackSonarB;
        uint8_t m_RightSonarA;
        uint8_t m_RightSonarB;        
        uint8_t m_Footer;
    };
    
    // Ensures a number is between the upper and lower bounds
    inline float Limit( float num, float upper, float lower );
    
    // Trims a number to be in between the upper and lower bounds
    inline float Trim( float num, float upper, float lower );

    // Detect that a button has been pressed (rather than released)
    inline bool DetectTriggerChange(TriggerChangeValues * pTriggerVals);

    // Function to check for drive control direction swap
    inline void CheckForDriveSwap();
    
    // Get a throttle control value from a joystick
    inline float GetThrottleControl(Joystick * pJoystick);
    
    // Convert a distance in inches to encoder turns
    int GetEncoderRotationsFromInches(int inches, float diameter, bool bUseQuadEncoding = true);

    // Grabs a value from a sonar sensor individually
    // inline float UpdateSonarSensor(Ultrasonic * pSensor);
   
    // Get a reading from the gyro sensor
    // inline float GetGyroAngle(AnalogGyro * pSensor);

    // Autonomous wait for something to complete delay routine
    inline void AutonomousDelay(float time);

    // Autonomous drive for a specified time
    inline void AutonomousDriveSequence(float speed, float time);
    
    // Autonomous routines to back drive the motors to abruptly stop
    inline void AutonomousBackDrive(EncoderDirection currentRobotDirection);
    inline void AutonomousBackDriveTurn(GyroDirection currentGyroDirection);
    
    // Autonomous routines
    void AutonomousRoutine1();
    void AutonomousRoutine2(bool bBoilerSide);
    void AutonomousRoutine3();
    void AutonomousCommon(bool bStartShoot, bool bEndShoot);
    void AutonomousCommonRed(float driveSpeed, bool bEndShoot);
    void AutonomousCommonBlue(float driveSpeed, bool bEndShoot);
    void AutonomousShoot();
    bool AutonomousGyroLeftTurn(float destAngle, float turnSpeed);
    bool AutonomousGyroRightTurn(float destAngle, float turnSpeed);
    void AutonomousEncoderDrive(float speed, float distance, EncoderDirection direction);
    bool AutonomousSonarDrive(SonarDriveDirection driveDirection, SonarDriveState driveState, uint32_t destLateralDist, uint32_t destSideDist);

    // Routine to put things in a known state
    void InitialStateSetup();

    // Main sequence for drive motor control
    void DriveControlSequence();
    
    // Functions to automate slightly moving the robot
    void DirectionalInch(float speed, EncoderDirection direction);

    // Main sequence for LED control
    void LedSequence();

    // Main sequence for updating solenoid states
    void SolenoidSequence();

    // Main sequence for grabbing values from the sonars
    void SonarSensorSequence();
    
    // Main sequence for reading gyro values and related processing
    void GyroSequence();

    // Main sequence for interaction with the serial port
    void SerialPortSequence();
    
    // Main sequence for I2C interaction
    void I2cSequence();

    // Main sequence for vision processing
    void CameraSequence();
    
    // Main sequence for fuel ball gathering
    void FuelIntakeSequence();
    
    // Main sequence for fuel pump processing
    void FuelPumpSequence();
    
    // Main sequence for the fuel injecting and shooting
    void FuelInjectAndShootSequence();
    
    // Main sequence for utilizing the agitator and flap
    void AgitatorAndFlapSequence();
    
    // Main sequence for gear operations
    void GearSequence();
    
    // Main sequence for lifting off the robot
    void LiftOffSequence();

    // Test routines for trying out experimental code
    void AutonomousTestCode();
    void OperatorTestCode();
    void MotorTest();
    void TankDrive();
    
    // MEMBER VARIABLES
    
    // User Controls
    DriverStation           *m_pDriverStation;              // Driver station object for getting selections
    Joystick                *m_pDriveJoystick;              // Drive control
    Joystick                *m_pControlJoystick;            // Robot functional control
    
    // Motors
    TalonMotorGroup         *m_pLeftDriveMotor;             // Left drive motor control
    TalonMotorGroup         *m_pRightDriveMotor;            // Right drive motor control
    TalonMotorGroup         *m_pGearIntakeMotor;            // Motor controlling gear intake  mechanism
    TalonMotorGroup         *m_pGearTiltMotor;              // Motor controlling gear intake tilt
    //TalonMotorGroup         *m_pFuelIntakeMotor;            // Fuel ball intake motor
    //TalonMotorGroup         *m_pFuelPumpMotor;              // Fuel ball pump motor
    TalonMotorGroup         *m_pFuelInjectorMotor;          // Fuel ball injector motor
    TalonMotorGroup         *m_pFuelShooterMotor;           // Fuel ball shooter motors
    TalonMotorGroup         *m_pFuelShooterAngleMotor;      // Fuel ball shooter angle adjustment motor
    TalonMotorGroup         *m_pLiftOffMotor;               // Motor to control raising the robot
    TalonMotorGroup         *m_pAgitatorMotor;              // Motor to control agitating the fuel balls
    TalonMotorGroup         *m_pFlapMotor;                  // Gear hang flap motor
    
    // Spike Relays
    Relay                   *m_pFlashLightRelay;            // Controls turning the flashlight on/off
    Relay                   *m_pLedRelay;                   // Controls whether or not the LEDs are lit up
    
    // Digital I/O
    DigitalInput            *m_pAutonomous1Switch;          // Switch to select autonomous routine 1
    DigitalInput            *m_pAutonomous2Switch;          // Switch to select autonomous routine 2
    DigitalInput            *m_pAutonomous3Switch;          // Switch to select autonomous routine 3
    //DigitalInput            *m_pMagneticSwitch;             // Magnetic switch digital input
    DigitalInput            *m_pShooterAngleUpLimitSwitch;  // Limit switch to shut off motor when shooter angle is max'ed
    DigitalInput            *m_pShooterAngleDownLimitSwitch;// Limit switch to shut off motor when shooter angle is min'ed
    DigitalInput            *m_pGearPressureSwitch1;        // 1 of 2 limit switches below the gear pressure plate
    DigitalInput            *m_pGearPressureSwitch2;        // 2 of 2 limit switches below the gear pressure plate
    DigitalInput            *m_pGearTiltForwardLimitSwitch; // Gear tilt forward limit switch
    DigitalInput            *m_pGearTiltReverseLimitSwitch; // Gear tilt reverse limit switch
    //DigitalInput            *m_pFlapForwardLimitSwitch;     // Flap limit switch forward
    //DigitalInput            *m_pFlapReverseLimitSwitch;     // Flap limit switch reverse
    
    // Analog I/O
    AnalogGyro              *m_pGyro;
    
    // Solenoids
    // Note: no compressor related objects required,
    // instantiating a solenoid gets that for us.
    // DoubleSolenoid       *
    
    // Servos
    // (none)
    
    // Encoders
    // (none)
    
    // Timers
    Timer                   *m_pAutonomousTimer;            // Time things during autonomous
    Timer                   *m_pInchingDriveTimer;          // Keep track of an inching drive operation
    Timer                   *m_pI2cTimer;                   // Keep track of how often to do I2C operations
    Timer                   *m_pCameraRunTimer;             // Keep track of how often to do camera intense code runs
    Timer                   *m_pFuelPumpTimer;              // Keep track of fuel pump related operations
    Timer                   *m_pGearTimer;                  // Keep track of gear operations
    Timer                   *m_pLiftOffTimer;               // Keep track of lift off related operations
    Timer                   *m_pAgitatorTimer;              // Keep track of how long the agitator is on
    Timer                   *m_pSafetyTimer;                // Fail safe in case critical operations don't complete
    
    // Accelerometer
    BuiltInAccelerometer    *m_pAccelerometer;              // Built in roborio accelerometer

    // Camera
    cs::UsbCamera           m_Cam0;
    cs::UsbCamera           m_Cam1;
    cs::VideoSink           m_CameraServer;
    //cs::CvSink              *m_pSink0;
    //cs::CvSink              *m_pSink1;
    //cv::Mat                 m_Mat;
    static const int        CAMERA_X_RES                           = 320;
    static const int        CAMERA_Y_RES                           = 240;

    // Serial port configuration
    static const int        SERIAL_PORT_BUFFER_SIZE_BYTES          = 1024;
    static const int        SERIAL_PORT_NUM_DATA_BITS              = 8;
    static const int        SERIAL_PORT_BAUD_RATE                  = 115200;
    static const int        ASCII_0_OFFSET                         = 48;
    const char *            SERIAL_PORT_PACKET_HEADER              = "Frc120";
    const int               SERIAL_PORT_PACKET_HEADER_SIZE_BYTES   = 6;
    char                    m_SerialPortBuffer[SERIAL_PORT_BUFFER_SIZE_BYTES];

    // On board serial port
    SerialPort              *m_pSerialPort;
    
    // I2C configuration
    static const int        I2C_DEVICE_ADDRESS                      = 4U;
    I2cData                 m_I2cData;
    I2C                     *m_pI2cPort;

    // Misc
    Alliance                m_AllianceColor;                        // Color reported by driver station during a match
    bool                    m_bDriveSwap;                           // Allow the user to push a button to change forward/reverse
    bool                    m_bLed;                                 // Keep track of turning an LED on/off
    PulseState              m_FuelPumpState;                        // Keep track of fuel pump pulsing state
    PulseState              m_LiftOffState;                         // Keep track of whether or not to pulse the lift off motor
    AgitatorState           m_AgitatorState;                        // Keep track of agitator state
    GearState               m_GearState;                            // Keep track of the gear intake state
        
    // Keep three recent readings from each sensor
    static const int        SONAR_DATA_INDEX_START                  = 0U;
    static const int        SONAR_DATA_INDEX_END                    = 2U;
    uint8_t                 m_SonarDataIndex;
    uint8_t                 m_RecentSonarValues[I2cData::NUM_SONAR_SENSORS * SONAR_DATA_INDEX_END];
    
    // CONSTS
    
    // Joysticks/Buttons
    static const int        DRIVE_JOYSTICK                          = 0;
    static const int        CONTROL_JOYSTICK                        = 1;

    // Driver buttons
    //static const int        DRIVER_FUEL_INTAKE_BUTTON               = 1;
    static const int        GEAR_INTAKE_FORWARD_BUTTON              = 1;
    static const int        GEAR_INTAKE_REVERSE_BUTTON              = 3;
    //static const int        AGITATOR_STIR_BUTTON                    = 2;
    static const int        ROBOT_LIFT_OFF_AUTO_PULSE_ON            = 5;//11
    static const int        ROBOT_LIFT_OFF_AUTO_PULSE_OFF           = 10;//12
    static const int        ROBOT_LIFT_OFF_DOWN_BUTTON              = 6;
    static const int        ROBOT_LIFT_OFF_UP_BUTTON                = 7;
    //static const int        FLASH_LIGHT_ON_BUTTON                   = 9;
    //static const int        FLASH_LIGHT_OFF_BUTTON                  = 8;
    //static const int        INCH_FORWARD_BUTTON                     = 11;
    //static const int        INCH_BACKWARD_BUTTON                    = 15;
    //static const int        INCH_LEFT_BUTTON                        = 3;//12;
    //static const int        INCH_RIGHT_BUTTON                       = 4;//16;
    static const int        DRIVE_CONTROLS_FORWARD_BUTTON           = 13;
    static const int        DRIVE_CONTROLS_REVERSE_BUTTON           = 14;
    
    // Control buttons
    static const int        FUEL_INJECT_AND_SHOOT_BUTTON            = 1;
    static const int        AGITATOR_STIR_BUTTON                    = 1;
    //static const int        FUEL_PUMP_BUTTON                        = 3;
    //static const int        FUEL_PUMP_PULSE_BUTTON                  = 4;
    //static const int        CONTROLLER_FUEL_INTAKE_BUTTON           = 5;
    //static const int        FUEL_PUMP_UNJAM_BUTTON                  = 6;
    static const int        GEAR_HANG_AUTO_BUTTON                   = 2;
    static const int        GEAR_HANG_FORWARD_BUTTON                = 3;
    static const int        GEAR_HANG_REVERSE_BUTTON                = 4;
    static const int        CAMERA_0_BUTTON                         = 7;
    static const int        CAMERA_1_BUTTON                         = 8;
    static const int        FLAP_OUT_BUTTON                         = 9;
    static const int        FLAP_IN_BUTTON                          = 10;
    static const int        ADJUST_SHOOTER_ANGLE_DOWN_BUTTON        = 11;
    static const int        ADJUST_SHOOTER_ANGLE_UP_BUTTON          = 12;
    static const int        ESTOP_BUTTON                            = 14;

    // CAN Signals
    static const int        LEFT_MOTORS_CAN_START_ID                = 1;
    static const int        RIGHT_MOTORS_CAN_START_ID               = 3;
    static const int        FUEL_SHOOTER_MOTORS_CAN_START_ID        = 5;
    static const int        FUEL_INJECTOR_MOTOR_CAN_ID              = 7;
    static const int        GEAR_TILT_CAN_ID                        = 8;
    static const int        GEAR_INTAKE_CAN_ID                      = 9;
    //static const int        FUEL_PUMP_MOTOR_CAN_ID                  = 8;
    //static const int        FUEL_INTAKE_MOTOR_CAN_ID                = 9;
    static const int        FUEL_SHOOTER_ANGLE_MOTOR_CAN_ID         = 10;
    static const int        LIFT_OFF_MOTORS_CAN_START_ID            = 11;
    static const int        FLAP_MOTOR_CAN_ID                       = 13;
    static const int        AGITATOR_MOTOR_CAN_ID                   = 14;

    // PWM Signals
    // (none)
    
    // Relays
    static const int        FLASHLIGHT_RELAY_ID                     = 0;
    static const int        LED_RELAY_ID                            = 3;
    
    // Digital I/O Signals
    static const int        AUTONOMOUS_1_SWITCH                     = 0;
    static const int        AUTONOMOUS_2_SWITCH                     = 1;
    static const int        AUTONOMOUS_3_SWITCH                     = 2;
    //static const int        MAGNETIC_SWITCH_INPUT_PIN               = 5;
    static const int        SHOOTER_ANGLE_UP_LIMIT_SWITCH           = 3;
    static const int        SHOOTER_ANGLE_DOWN_LIMIT_SWITCH         = 4;
    static const int        GEAR_PRESSURE_SWITCH_1                  = 7;
    static const int        GEAR_PRESSURE_SWITCH_2                  = 8;
    static const int        GEAR_TILT_FORWARD_LIMIT_SWITCH          = 6;
    static const int        GEAR_TILT_REVERSE_LIMIT_SWITCH          = 5;
    //static const int        FLAP_FORWARD_LIMIT_SWITCH               = 9;
    //static const int        FLAP_REVERSE_LIMIT_SWITCH               = 10;
    
    // Analog I/O Signals
    static const int        ANALOG_GYRO_CHANNEL                     = 0;
    
    // Solenoid Signals
    // (none)
    
    // Misc
    static const int        OFF                                     = 0;
    static const int        ON                                      = 1;
    static const int        SINGLE_MOTOR                            = 1;
    static const int        NUMBER_OF_LEFT_DRIVE_MOTORS             = 2;
    static const int        NUMBER_OF_RIGHT_DRIVE_MOTORS            = 2;
    static const int        NUMBER_OF_FUEL_SHOOTER_MOTORS           = 2;
    static const int        NUMBER_OF_LIFT_OFF_MOTORS               = 2;
    static const int        SCALE_TO_PERCENT                        = 100;
    static const int        QUADRATURE_ENCODING_ROTATIONS           = 4096;
    static const char       NULL_CHARACTER                          = '\0';
    static const bool       DEBUG_PRINTS                            = true;
    
    static constexpr float  JOYSTICK_TRIM_UPPER_LIMIT               =  0.10F;
    static constexpr float  JOYSTICK_TRIM_LOWER_LIMIT               = -0.10F;
    static constexpr float  CONTROL_THROTTLE_VALUE_RANGE            =  0.65F;
    static constexpr float  CONTROL_THROTTLE_VALUE_BASE             =  0.35F;
    static constexpr float  DRIVE_THROTTLE_VALUE_RANGE              =  0.65F;
    static constexpr float  DRIVE_THROTTLE_VALUE_BASE               =  0.35F;
    static constexpr float  DRIVE_WHEEL_DIAMETER_INCHES             =  4.00F;
    static constexpr float  DRIVE_MOTOR_UPPER_LIMIT                 =  1.00F;
    static constexpr float  DRIVE_MOTOR_LOWER_LIMIT                 = -1.00F;
    static constexpr float  MOTOR_BACK_DRIVE_SPEED                  =  0.05F;
    static constexpr float  INCHING_DRIVE_SPEED                     =  0.25F;
    static constexpr float  INCHING_DRIVE_DELAY_S                   =  0.10F;
    
    static constexpr float  FUEL_INTAKE_SCALING                     =  0.75F;
    static constexpr float  FUEL_PUMP_SCALING_FORWARD               =  0.50F;
    static constexpr float  FUEL_PUMP_SCALING_REVERSE               =  0.25F;
    static constexpr float  FUEL_PUMP_SPACING_DELAY_S               =  0.25F;
    static constexpr float  FUEL_PUMP_SAFETY_TIMER_MAX_VALUE        =  1.00F;
    static constexpr float  SHOOTER_ANGLE_SCALING                   =  0.15F;
    static constexpr float  GEAR_TILT_FORWARD_SCALING               =  0.20F;
    static constexpr float  GEAR_TILT_REVERSE_SCALING               =  0.50F;
    static constexpr float  GEAR_INTAKE_SCALING                     =  0.50F;
    static constexpr float  GEAR_SETTLING_TIME_S                    =  1.00F;
    static constexpr float  LIFT_OFF_MOTOR_SPEED_OFFSET             =  0.04F;
    static constexpr float  LIFT_OFF_AUTO_PULSE_DELAY_S             =  0.75F;
    static constexpr float  AGITATOR_SCALING                        =  0.75F;
    static constexpr float  MAX_AGITATION_TIME_S                    =  3.00F;
    static constexpr float  MAX_AGITATION_REFRACTORY_TIME_S         =  1.50F;
    static constexpr float  CAMERA_RUN_INTERVAL_S                   =  1.00F;
    static constexpr float  I2C_RUN_INTERVAL_S                      =  0.10F;
    static constexpr float  SAFETY_TIMER_MAX_VALUE                  =  5.00F;
    
    static const int        ENCODER_TURNS_PER_FUEL_PUMP_PULSE       = 3911 / 4;
    
    static const int        SONAR_LED_WARN_DIST_INCHES              = 3;
    static const uint32_t   SONAR_DRIVE_STATE_SIDE_MASK             = 0x0F;
    static const uint32_t   SONAR_DRIVE_STATE_LATERAL_MASK          = 0xF0;
    
};  // End class



////////////////////////////////////////////////////////////////
// @method CmsdRobot::CheckForDriveSwap
///
/// Updates the drive control direction.
///
////////////////////////////////////////////////////////////////
inline void CmsdRobot::CheckForDriveSwap()
{
    // Check if the driver pushed the button to have
    // forward be reverse and vice versa
    if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_FORWARD_BUTTON) )
    {
        m_bDriveSwap = false;
    }
    else if ( m_pDriveJoystick->GetRawButton(DRIVE_CONTROLS_REVERSE_BUTTON) )
    {
        m_bDriveSwap = true;
    }
    else
    {
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::GetThrottleControl
///
/// Returns a throttle value based on input from the joystick.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::GetThrottleControl(Joystick * pJoystick)
{
    // Get throttle control
    // The z axis goes from -1 to 1, so it needs to be normalized.
    // Subtract one and negate to make it zero based to give a value
    // between zero and two.  This will be used to scale the voltage
    // to the motors.  It essentially computes the max speed value
    // that can be reached.
    if (pJoystick == m_pDriveJoystick)
    {
        return ((pJoystick->GetThrottle() - 1.0F) / -2.0F) * DRIVE_THROTTLE_VALUE_RANGE + DRIVE_THROTTLE_VALUE_BASE;
    }
    else
    {
        return ((pJoystick->GetThrottle() - 1.0F) / -2.0F) * CONTROL_THROTTLE_VALUE_RANGE + CONTROL_THROTTLE_VALUE_BASE;
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::GetGyroAngle
///
/// This method is used to get a value from an analog gyro sensor.
///
////////////////////////////////////////////////////////////////
/*
inline float CmsdRobot::GetGyroAngle(AnalogGyro * pSensor)
{
    return pSensor->GetAngle();
}
*/



////////////////////////////////////////////////////////////////
// @method CmsdRobot::UpdateSonarSensor
///
/// This method is used to get a value from the sonar sensor.
/// It is intended to be used to turn a sensor briefly on and
/// get a reading from it so as to not interfere with other
/// sensors that may need to get readings.
///
////////////////////////////////////////////////////////////////
/*
inline float CmsdRobot::UpdateSonarSensor(Ultrasonic * pSensor)
{
    pSensor->SetEnabled(true);
    float sensorValue = pSensor->GetRangeInches();
    pSensor->SetEnabled(false);
    return sensorValue;
}
*/


////////////////////////////////////////////////////////////////
// @method CmsdRobot::Limit
///
/// This method is used to prevent a value outside the range
/// specified by upper and lower from being sent to physical
/// devices.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::Limit( float num, float upper, float lower )
{
    if ( num > upper )
    {
        return upper;
    }
    else if ( num < lower )
    {
        return lower;
    }

    return num;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Trim
///
/// This method is used to ensure a signal value is above a
/// certain threshold to ensure there is actual input to a
/// device and not just noise/jitter.
///
////////////////////////////////////////////////////////////////
inline float CmsdRobot::Trim( float num, float upper, float lower )
{
    if ( (num < upper) && (num > lower) )
    {
        return 0;
    }
    
    return num;
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::DetectTriggerChange
///
/// This method is used to check if a button has undergone a
/// state change.  The same button can be used to reverse state
/// of a particular part of the robot (such as a motor or
/// solenoid).  If the state is reversed inside an 'if'
/// statement that is part of a loop, the final state will be
/// whatever transition just occurred, which could be back to
/// the same state started in.  The intended use case is to
/// have TriggerChangeValues variables in the code and update
/// their 'current' value each time through the loop by reading
/// the joystick input.  This input will then be checked against
/// the old input and return 'true' if it detects the button has
/// been pressed.  This method is intended to be called inside
/// 'if' statements for logic control.
///
////////////////////////////////////////////////////////////////
inline bool CmsdRobot::DetectTriggerChange(TriggerChangeValues * pTriggerVals)
{
    // Only report a change if the current value is different than the old value
    // Also make sure the transition is to being pressed since we are detecting
    // presses and not releases
    if ( (pTriggerVals->bCurrentValue != pTriggerVals->bOldValue) && pTriggerVals->bCurrentValue )
    {
        // Update the old value, return the button was pressed
        pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
        return true;
    }
    
    // Otherwise update the old value
    pTriggerVals->bOldValue = pTriggerVals->bCurrentValue;
    return false;
}

#endif // CMSDROBOT_HPP
