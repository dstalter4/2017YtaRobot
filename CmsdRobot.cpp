////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobot.cpp
///
/// Implementation of the CmsdRobot class.  This file contains the functions
/// for full robot operation in FRC.  It contains the autonomous and operator
/// control routines as well as all necessary support for interacting with all
/// motors, sensors and input/outputs on the robot.
///
/// CMSD FRC 2017
/// Author(s): David Stalter
/// @Edit History
/// - dts   09-JAN-2016 Created from 2015.
/// - dts   08-JAN-2017 Ported from 2016.
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
#include <cstddef>          // for nullptr
#include <cstring>          // for memset

// C INCLUDES
// (none)

// C++ INCLUDES
#include "CmsdRobot.hpp"    // For class declaration (and other headers)

// Do not use static initialization!  There is a bug in the
// WPI libraries that will cause an exception during object
// instantiation for the robot.


////////////////////////////////////////////////////////////////
// @method CmsdRobot::CmsdRobot
///
/// Constructor.  Instantiates all robot control objects.
///
////////////////////////////////////////////////////////////////
CmsdRobot::CmsdRobot()
: m_pDriverStation              (&DriverStation::GetInstance())
, m_pDriveJoystick              (new Joystick(DRIVE_JOYSTICK))
, m_pControlJoystick            (new Joystick(CONTROL_JOYSTICK))
, m_pLeftDriveMotor             (new TalonMotorGroup(NUMBER_OF_LEFT_DRIVE_MOTORS, LEFT_MOTORS_CAN_START_ID, ControlMode::FOLLOW, FeedbackDevice::CtreMagEncoder_Relative))
, m_pRightDriveMotor            (new TalonMotorGroup(NUMBER_OF_RIGHT_DRIVE_MOTORS, RIGHT_MOTORS_CAN_START_ID, ControlMode::FOLLOW, FeedbackDevice::CtreMagEncoder_Relative))
, m_pGearIntakeMotor            (new TalonMotorGroup(SINGLE_MOTOR, GEAR_INTAKE_CAN_ID, ControlMode::INDEPENDENT))
, m_pGearTiltMotor              (new TalonMotorGroup(SINGLE_MOTOR, GEAR_TILT_CAN_ID, ControlMode::INDEPENDENT))
//, m_pFuelIntakeMotor            (new TalonMotorGroup(SINGLE_MOTOR, FUEL_INTAKE_MOTOR_CAN_ID, ControlMode::INDEPENDENT))
//, m_pFuelPumpMotor              (new TalonMotorGroup(SINGLE_MOTOR, FUEL_PUMP_MOTOR_CAN_ID, ControlMode::INDEPENDENT, FeedbackDevice::CtreMagEncoder_Relative))
, m_pFuelInjectorMotor          (new TalonMotorGroup(SINGLE_MOTOR, FUEL_INJECTOR_MOTOR_CAN_ID, ControlMode::INDEPENDENT))
, m_pFuelShooterMotor           (new TalonMotorGroup(NUMBER_OF_FUEL_SHOOTER_MOTORS, FUEL_SHOOTER_MOTORS_CAN_START_ID, ControlMode::FOLLOW))
, m_pFuelShooterAngleMotor      (new TalonMotorGroup(SINGLE_MOTOR, FUEL_SHOOTER_ANGLE_MOTOR_CAN_ID, ControlMode::INDEPENDENT))
, m_pLiftOffMotor               (new TalonMotorGroup(NUMBER_OF_LIFT_OFF_MOTORS, LIFT_OFF_MOTORS_CAN_START_ID, ControlMode::INVERSE_OFFSET))
, m_pAgitatorMotor              (new TalonMotorGroup(SINGLE_MOTOR, AGITATOR_MOTOR_CAN_ID, ControlMode::INDEPENDENT))
, m_pFlapMotor                  (new TalonMotorGroup(SINGLE_MOTOR, FLAP_MOTOR_CAN_ID, ControlMode::INDEPENDENT))
, m_pFlashLightRelay            (new Relay(FLASHLIGHT_RELAY_ID))
, m_pLedRelay                   (new Relay(LED_RELAY_ID))
, m_pAutonomous1Switch          (new DigitalInput(AUTONOMOUS_1_SWITCH))
, m_pAutonomous2Switch          (new DigitalInput(AUTONOMOUS_2_SWITCH))
, m_pAutonomous3Switch          (new DigitalInput(AUTONOMOUS_3_SWITCH))
//, m_pMagneticSwitch             (new DigitalInput(MAGNETIC_SWITCH_INPUT_PIN))
, m_pShooterAngleUpLimitSwitch  (new DigitalInput(SHOOTER_ANGLE_UP_LIMIT_SWITCH))
, m_pShooterAngleDownLimitSwitch(new DigitalInput(SHOOTER_ANGLE_DOWN_LIMIT_SWITCH))
, m_pGearPressureSwitch1        (new DigitalInput(GEAR_PRESSURE_SWITCH_1))
, m_pGearPressureSwitch2        (new DigitalInput(GEAR_PRESSURE_SWITCH_2))
, m_pGearTiltForwardLimitSwitch (new DigitalInput(GEAR_TILT_FORWARD_LIMIT_SWITCH))
, m_pGearTiltReverseLimitSwitch (new DigitalInput(GEAR_TILT_REVERSE_LIMIT_SWITCH))
//, m_pFlapForwardLimitSwitch     (new DigitalInput(FLAP_FORWARD_LIMIT_SWITCH))
//, m_pFlapReverseLimitSwitch     (new DigitalInput(FLAP_REVERSE_LIMIT_SWITCH))
, m_pGyro                       (new AnalogGyro(ANALOG_GYRO_CHANNEL))
, m_pAutonomousTimer            (new Timer())
, m_pInchingDriveTimer          (new Timer())
, m_pI2cTimer                   (new Timer())
, m_pCameraRunTimer             (new Timer())
, m_pFuelPumpTimer              (new Timer())
, m_pGearTimer                  (new Timer())
, m_pLiftOffTimer               (new Timer())
, m_pAgitatorTimer              (new Timer())
, m_pSafetyTimer                (new Timer())
, m_pAccelerometer              (new BuiltInAccelerometer())
, m_Cam0                        (CameraServer::GetInstance()->StartAutomaticCapture(0U))
, m_Cam1                        (CameraServer::GetInstance()->StartAutomaticCapture(1U))
, m_CameraServer                (CameraServer::GetInstance()->GetServer())
//, m_pSink0                      (new cs::CvSink("cam0sink"))
//, m_pSink1                      (new cs::CvSink("cam1sink"))
//, m_Mat                         ()
, m_SerialPortBuffer            ()
, m_pSerialPort                 (new SerialPort(SERIAL_PORT_BAUD_RATE, SerialPort::kMXP, SERIAL_PORT_NUM_DATA_BITS, SerialPort::kParity_None, SerialPort::kStopBits_One))
, m_I2cData                     ()
, m_pI2cPort                    (new I2C(I2C::kMXP, I2C_DEVICE_ADDRESS))
, m_AllianceColor               (m_pDriverStation->GetAlliance())
, m_bDriveSwap                  (false)
, m_bLed                        (false)
, m_FuelPumpState               (MANUAL_PULSE)
, m_LiftOffState                (MANUAL_PULSE)
, m_AgitatorState               (READY)
, m_GearState                   (MANUAL_INTAKE)
, m_SonarDataIndex              (SONAR_DATA_INDEX_START)
, m_RecentSonarValues           ()
{
    // Clear sonar related arrays
    static_cast<void>(std::memset(m_RecentSonarValues, 0, sizeof(m_RecentSonarValues)));
    
    // Reset all timers
    m_pAutonomousTimer->Reset();
    m_pInchingDriveTimer->Reset();
    m_pI2cTimer->Reset();
    m_pCameraRunTimer->Reset();
    m_pFuelPumpTimer->Reset();
    m_pGearTimer->Reset();
    m_pLiftOffTimer->Reset();
    m_pAgitatorTimer->Reset();
    m_pSafetyTimer->Reset();

    // Reset the serial port
    m_pSerialPort->Reset();

    // Set camera quality
    m_Cam0.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    m_Cam1.SetResolution(CAMERA_X_RES, CAMERA_Y_RES);
    
    //m_Cam0.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 10);
    //m_Cam1.SetVideoMode(cs::VideoMode::kMJPEG, 160, 120, 10);
    
    //m_pSink0->SetSource(m_Cam0);
    //m_pSink0->SetEnabled(true);
    //m_pSink1->SetSource(m_Cam1);
    //m_pSink1->SetEnabled(true);
    m_CameraServer.SetSource(m_Cam0);
    m_pCameraRunTimer->Start();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::InitialStateSetup
///
/// This method contains the work flow for putting motors,
/// solenoids, etc. into a known state.  It is intended to be
/// used by both autonomous and user control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::InitialStateSetup()
{
    // Start with motors off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    m_pFuelShooterMotor->Set(OFF);
    m_pFuelInjectorMotor->Set(OFF);
    //m_pFuelPumpMotor->Set(OFF);
    //m_pFuelIntakeMotor->Set(OFF);
    m_pFuelShooterAngleMotor->Set(OFF);
    m_pLiftOffMotor->Set(OFF);
    m_pAgitatorMotor->Set(OFF);
    m_pGearIntakeMotor->Set(OFF);
    m_pGearTiltMotor->Set(OFF);
    m_pFlapMotor->Set(OFF);
    
    // Tare encoders
    m_pLeftDriveMotor->TareEncoder();
    m_pRightDriveMotor->TareEncoder();
    
    // Stop/clear any timers, just in case
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
    m_pFuelPumpTimer->Stop();
    m_pFuelPumpTimer->Reset();
    m_pGearTimer->Stop();
    m_pGearTimer->Reset();
    m_pLiftOffTimer->Stop();
    m_pLiftOffTimer->Reset();
    m_pAgitatorTimer->Stop();
    m_pAgitatorTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Start I2C timer
    m_pI2cTimer->Reset();
    m_pI2cTimer->Start();
    
    // Just in case constructor was called before this was set
    m_AllianceColor = m_pDriverStation->GetAlliance();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::OperatorControl
///
/// The user control method.  This method is called once each
/// time the robot enters operator control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::OperatorControl()
{
    // Autonomous should have left things in a known state, but
    // just in case clear everything.  Timers were reset in the
    // constructor, no need to do it again
    InitialStateSetup();
    
    // Teleop will use coast mode
    //m_pLeftDriveMotor->SetCoastMode();
    //m_pRightDriveMotor->SetCoastMode();
    // Teleop will use brake mode
    m_pLeftDriveMotor->SetBrakeMode();
    m_pRightDriveMotor->SetBrakeMode();
    
    // Main tele op loop
    while ( m_pDriverStation->IsOperatorControl() )
    {
        //CheckForDriveSwap();

        DriveControlSequence();
        
        //LedSequence();

        //SolenoidSequence();

        //SonarSensorSequence();
        
        GyroSequence();

        //SerialPortSequence();
        
        I2cSequence();
        
        CameraSequence();
        
        AgitatorAndFlapSequence();
        
        //FuelIntakeSequence();
        //FuelPumpSequence();
        FuelInjectAndShootSequence();
        
        GearSequence();
        
        LiftOffSequence();
        
        
        
        // TEST CODE
        // Recommended to only enable this in test scenarios
        // to not impact matches
        //OperatorTestCode();
        //MotorTest();
        // END TEST CODE
        
    } // End main OperatorControl loop
} // End OperatorControl



////////////////////////////////////////////////////////////////
// @method CmsdRobot::FuelIntakeSequence
///
/// This method contains the main workflow for priming the
/// injector and shooter with balls via the fuel pump.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::FuelIntakeSequence()
{
    /*
    // First check for intake
    if (m_pDriveJoystick->GetRawButton(DRIVER_FUEL_INTAKE_BUTTON) || m_pControlJoystick->GetRawButton(CONTROLLER_FUEL_INTAKE_BUTTON))
    {
        m_pFuelIntakeMotor->Set(-ON * FUEL_INTAKE_SCALING);
    }
    else
    {
        m_pFuelIntakeMotor->Set(OFF);
    }
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::FuelPumpSequence
///
/// This method contains the main workflow for priming the
/// injector and shooter with balls via the fuel pump.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::FuelPumpSequence()
{
    /*
    static bool bForwardPulse = false;
    static bool bReversePulse = false;
    
    // Now check fuel ball movement inputs.  Start with the pump.
    switch (m_FuelPumpState)
    {
        // Fuel pump can be in three states: manual control, pulsing, pulse delay
        case MANUAL_PULSE:
        {
            // Check first for manual control
            if (m_pControlJoystick->GetRawButton(FUEL_PUMP_BUTTON))
            {
                m_pFuelPumpMotor->Set(ON);
            }
            // Now check for a pulse
            else if (m_pControlJoystick->GetRawButton(FUEL_PUMP_PULSE_BUTTON))
            {
                // Tare encoder, change state machine, start the timer
                m_pFuelPumpMotor->TareEncoder();
                m_FuelPumpState = PULSE_IN_PROGRESS;
                m_pFuelPumpMotor->Set(ON * FUEL_PUMP_SCALING_FORWARD);
                bForwardPulse = true;
                m_pSafetyTimer->Reset();
                m_pSafetyTimer->Start();
            }
            // Last check for unjam
            else if (m_pControlJoystick->GetRawButton(FUEL_PUMP_UNJAM_BUTTON))
            {
                m_pFuelPumpMotor->TareEncoder();
                m_FuelPumpState = PULSE_IN_PROGRESS;
                m_pFuelPumpMotor->Set(-ON * FUEL_PUMP_SCALING_REVERSE);
                bReversePulse = true;
                m_pSafetyTimer->Reset();
                m_pSafetyTimer->Start();
            }
            // No input, shut the motor off
            else
            {
                m_pFuelPumpMotor->Set(OFF);
            }
            break;
        }
        // A pulse is in progress, monitor the encoder and safety timer
        case PULSE_IN_PROGRESS:
        {
            if (bForwardPulse)
            {
                // Encoder counting up, if greater than positive turns, stop
                if ((m_pFuelPumpMotor->GetEncoderValue() >= ENCODER_TURNS_PER_FUEL_PUMP_PULSE) || (m_pSafetyTimer->Get() > FUEL_PUMP_SAFETY_TIMER_MAX_VALUE))
                {
                    // Change state to abruptly end the pulse
                    m_pFuelPumpMotor->Set(-ON * MOTOR_BACK_DRIVE_SPEED);
                }
            }
            else if (bReversePulse)
            {
                // Encoder counting down, if less than negative turns, stop
                if ((m_pFuelPumpMotor->GetEncoderValue() <= (-ENCODER_TURNS_PER_FUEL_PUMP_PULSE)) || (m_pSafetyTimer->Get() > FUEL_PUMP_SAFETY_TIMER_MAX_VALUE))
                {
                    // Change state to abruptly end the pulse
                    m_pFuelPumpMotor->Set(ON * MOTOR_BACK_DRIVE_SPEED);
                }
            }
            else
            {
            }
            
            // Safety timer off, fuel pump space timer on
            m_pSafetyTimer->Stop();
            m_pSafetyTimer->Reset();
            m_pFuelPumpTimer->Reset();
            m_pFuelPumpTimer->Start();
            
            m_FuelPumpState = PULSE_FINISHED_DELAY;
            
            break;
        }
        case PULSE_FINISHED_DELAY:
        {
            if (m_pFuelPumpTimer->Get() >= FUEL_PUMP_SPACING_DELAY_S)
            {
                m_FuelPumpState = MANUAL_PULSE;
                m_pFuelPumpMotor->Set(OFF);
                m_pFuelPumpTimer->Stop();
                m_pFuelPumpTimer->Reset();
                bForwardPulse = false;
                bReversePulse = false;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    
    if (DEBUG_PRINTS)
    {
        // 2017: Magnetic encoder is an open value of one.
        SmartDashboard::PutNumber("Pump encoder value: ", m_pFuelPumpMotor->GetEncoderValue());
        SmartDashboard::PutNumber("Magnetic switch value: ", m_pMagneticSwitch->Get());
    }
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::FuelInjectAndShootSequence
///
/// This method contains the main workflow for gathering fuel
/// balls, injecting them to the shoot, and shooting them.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::FuelInjectAndShootSequence()
{
    float throttleControl = GetThrottleControl(m_pControlJoystick);
        
    // Then the injector/shoot input (one button as outputs are intended to be joint)
    if (m_pControlJoystick->GetRawButton(FUEL_INJECT_AND_SHOOT_BUTTON))
    {
        m_pFuelInjectorMotor->Set(throttleControl * ON);
        m_pFuelShooterMotor->Set(throttleControl * -ON);
    }
    else
    {
        m_pFuelInjectorMotor->Set(OFF);
        m_pFuelShooterMotor->Set(OFF);
    }
    
    // Limit switches are normally open
    if (m_pControlJoystick->GetRawButton(ADJUST_SHOOTER_ANGLE_DOWN_BUTTON) && m_pShooterAngleDownLimitSwitch->Get())
    {
        m_pFuelShooterAngleMotor->Set(ON * SHOOTER_ANGLE_SCALING);
    }
    else if (m_pControlJoystick->GetRawButton(ADJUST_SHOOTER_ANGLE_UP_BUTTON) && m_pShooterAngleUpLimitSwitch->Get())
    {
        m_pFuelShooterAngleMotor->Set(-ON * SHOOTER_ANGLE_SCALING);
    }
    else
    {
        m_pFuelShooterAngleMotor->Set(OFF);
    }
    
    SmartDashboard::PutNumber("Shooter motor speed: ", throttleControl * ON * SCALE_TO_PERCENT);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AgitatorAndFlapSequence
///
/// This method contains the main workflow for the agitator.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AgitatorAndFlapSequence()
{
    // Check flap input
    if (m_pControlJoystick->GetRawButton(FLAP_OUT_BUTTON))
    {
        m_pFlapMotor->Set(ON);
    }
    else if (m_pControlJoystick->GetRawButton(FLAP_IN_BUTTON))
    {
        m_pFlapMotor->Set(-ON);
    }
    else
    {
        m_pFlapMotor->Set(OFF);
    }
    
    // Check agitator input
    if (m_pControlJoystick->GetRawButton(AGITATOR_STIR_BUTTON))
    {
        m_pAgitatorMotor->Set(-ON * AGITATOR_SCALING);
    }
    else
    {
        m_pAgitatorMotor->Set(OFF);
    }
    
    /*
    switch (m_AgitatorState)
    {
        case READY:
        {
            if (m_pDriveJoystick->GetRawButton(AGITATOR_STIR_BUTTON))
            {
                m_pAgitatorMotor->Set(ON * AGITATOR_SCALING);
                m_pAgitatorTimer->Start();
                m_AgitatorState = AGITATING;
            }
            break;
        }
        case AGITATING:
        {
            if (m_pAgitatorTimer->Get() > MAX_AGITATION_TIME_S)
            {
                m_pAgitatorMotor->Set(OFF);
                m_pAgitatorTimer->Reset();
                m_AgitatorState = REFRACTORY;
            }
            break;
        }
        case REFRACTORY:
        {
            if (m_pAgitatorTimer->Get() > MAX_AGITATION_REFRACTORY_TIME_S)
            {
                m_pAgitatorTimer->Stop();
                m_pAgitatorTimer->Reset();
                m_AgitatorState = READY;
            }
            break;
        }
        default:
        {
            break;
        }
    }
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::GearSequence
///
/// This method contains the main workflow for hanging a gear.
/// It should be handled automatically by the driver, but in
/// case something is needed, there's a motor to open/close a
/// control flap.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::GearSequence()
{
    switch (m_GearState)
    {
        case MANUAL_INTAKE:
        {
            // First the gear intake buttons
            if (m_pDriveJoystick->GetRawButton(GEAR_INTAKE_FORWARD_BUTTON))
            {
                if (m_pGearPressureSwitch1->Get() && m_pGearPressureSwitch2->Get())
                {
                    m_pGearIntakeMotor->Set(ON * GEAR_INTAKE_SCALING);
                }
                else
                {
                    m_pGearIntakeMotor->Set(OFF);
                    m_pGearTimer->Reset();
                    m_pGearTimer->Start();
                    m_GearState = SETTLING;
                }
            }
            else if (m_pDriveJoystick->GetRawButton(GEAR_INTAKE_REVERSE_BUTTON))
            {
                m_pGearIntakeMotor->Set(-ON * GEAR_INTAKE_SCALING);
            }
            else
            {
                m_pGearIntakeMotor->Set(OFF);
            }
            
            // Then the gear tilt buttons
            if (m_pControlJoystick->GetRawButton(GEAR_HANG_FORWARD_BUTTON) && m_pGearTiltForwardLimitSwitch->Get())
            {
                m_pGearTiltMotor->Set(ON * GEAR_TILT_FORWARD_SCALING);
            }
            else if (m_pControlJoystick->GetRawButton(GEAR_HANG_REVERSE_BUTTON))// && m_pGearTiltReverseLimitSwitch->Get())
            {
                if (m_pGearTiltReverseLimitSwitch->Get())
                {
                    m_pGearTiltMotor->Set(-ON * GEAR_TILT_REVERSE_SCALING);
                }
                else
                {
                    m_pGearTiltMotor->Set(OFF);
                    m_pGearTimer->Reset();
                    m_pGearTimer->Start();
                    m_GearState = SETTLING;
                }
            }
            else
            {
                m_pGearTiltMotor->Set(OFF);
            }
            
            // Check for gear auto hang input
            if (m_pControlJoystick->GetRawButton(GEAR_HANG_AUTO_BUTTON))
            {
                m_pGearIntakeMotor->Set(ON * GEAR_INTAKE_SCALING);
                m_pGearTiltMotor->Set(ON * GEAR_TILT_FORWARD_SCALING);
                m_GearState = AUTO_INTAKE;
            }
            
            break;
        }
        case AUTO_INTAKE:
        {
            if (!m_pGearTiltForwardLimitSwitch->Get())
            {
                m_pGearIntakeMotor->Set(OFF);
                m_pGearTiltMotor->Set(OFF);
                m_GearState = MANUAL_INTAKE;
            }
            
            break;
        }
        case SETTLING:
        {
            if (m_pGearTimer->Get() >= GEAR_SETTLING_TIME_S)
            {
                m_pGearTimer->Stop();
                m_pGearTimer->Reset();
                m_GearState = MANUAL_INTAKE;
            }                
            break;
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::LiftOffSequence
///
/// This method contains the main workflow for lifting off the
/// robot in the end game.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::LiftOffSequence()
{
    static bool bLifting = false;
    
    // Most of the time things will be manual control, check that first
    switch (m_LiftOffState)
    {
        case MANUAL_PULSE:
        {
            // Going up
            if (m_pDriveJoystick->GetRawButton(ROBOT_LIFT_OFF_UP_BUTTON))
            {
                m_pLiftOffMotor->SetWithOffset(-ON, -(ON - LIFT_OFF_MOTOR_SPEED_OFFSET));
            }
            /*
            // Going down
            else if (m_pDriveJoystick->GetRawButton(ROBOT_LIFT_OFF_DOWN_BUTTON))
            {
                m_pLiftOffMotor->SetWithOffset(ON, ON - LIFT_OFF_MOTOR_SPEED_OFFSET);
            }
            */
            // Turn the auto pulse on
            else if (m_pDriveJoystick->GetRawButton(ROBOT_LIFT_OFF_AUTO_PULSE_ON))
            {
                // Turn the motors on, start the timer, update state machine
                m_pLiftOffMotor->SetWithOffset(-ON, -(ON - LIFT_OFF_MOTOR_SPEED_OFFSET));
                m_pLiftOffTimer->Reset();
                m_pLiftOffTimer->Start();
                m_LiftOffState = PULSE_IN_PROGRESS;
                bLifting = true;
            }
            // Nothing pressed, motors off
            else
            {
                m_pLiftOffMotor->SetWithOffset(OFF, OFF);
            }
            break;
        }
        // Auto pulse is in progress
        case PULSE_IN_PROGRESS:
        {
            // Check if the driver wants to shut it off
            if (m_pDriveJoystick->GetRawButton(ROBOT_LIFT_OFF_AUTO_PULSE_OFF))
            {
                // Motors off, timer off, state machine to manual
                m_pLiftOffMotor->SetWithOffset(OFF, OFF);
                m_pLiftOffTimer->Stop();
                m_pLiftOffTimer->Reset();
                m_LiftOffState = MANUAL_PULSE;
            }
            // Otherwise look for a timer trip to change state
            else
            {
                if (m_pLiftOffTimer->Get() >= LIFT_OFF_AUTO_PULSE_DELAY_S)
                {
                    if (bLifting)
                    {
                        m_pLiftOffMotor->SetWithOffset(OFF, OFF);
                    }
                    else
                    {
                        m_pLiftOffMotor->SetWithOffset(-ON, -(ON - LIFT_OFF_MOTOR_SPEED_OFFSET));
                    }
                    
                    // Restart the timer, change the lifting state
                    m_pLiftOffTimer->Reset();
                    bLifting = !bLifting;
                }
            }
        }
        default:
        {
            break;
        }
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::LedSequence
///
/// This method contains the main workflow for controlling
/// any LEDs on the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::LedSequence()
{
    // If the target's in range, give a visual indication
    if (m_bLed)
    {
        m_pLedRelay->Set(RelayValue::kOn);
    }
    else
    {
        // Otherwise set them off
        m_pLedRelay->Set(RelayValue::kOff);
    }
    
    /*
    if (m_pDriveJoystick->GetRawButton(FLASH_LIGHT_ON_BUTTON))
    {
        m_pFlashLightRelay->Set(RelayValue::kForward);
    }
    else if (m_pDriveJoystick->GetRawButton(FLASH_LIGHT_OFF_BUTTON))
    {
        m_pFlashLightRelay->Set(RelayValue::kOff);
    }
    else
    {
    }
    */
    
    /*
    if (m_pAutonomousRoutine1Switch->Get() || m_pAutonomousRoutine2Switch->Get() || m_pAutonomousRoutine3Switch->Get())
    {
        m_pFlashLightRelay->Set(RelayValue::kForward);
    }
    else
    {
        m_pFlashLightRelay->Set(RelayValue::kOff);
    }
    if (!m_pMagneticSwitch->Get())
    {
        m_pFlashLightRelay->Set(RelayValue::kForward);
    }
    else
    {
        m_pFlashLightRelay->Set(RelayValue::kOff);
    }
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::SolenoidSequence
///
/// This method contains the main workflow for updating the
/// state of the solenoids on the robot.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SolenoidSequence()
{
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::SonarSensorSequence
///
/// This method contains the main workflow for getting updates
/// from the sonar sensors.  In order to not interfere with
/// each other, each sensor is enabled/disabled and checked
/// individually.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SonarSensorSequence()
{
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::GyroSequence
///
/// This method contains the main workflow for getting updates
/// from the gyro sensors and processing related to those
/// readings.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::GyroSequence()
{
    if (DEBUG_PRINTS)
    {
        SmartDashboard::PutNumber("Gyro: ", m_pGyro->GetAngle());
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::SerialPortSequence
///
/// This method contains the main workflow for interaction with
/// the serial port.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::SerialPortSequence()
{
/*
    // Check for any incoming transmissions, limit it to our read buffer size
    int32_t bytesReceived = m_pSerialPort->GetBytesReceived();
    bytesReceived = (bytesReceived > SERIAL_PORT_BUFFER_SIZE_BYTES) ? SERIAL_PORT_BUFFER_SIZE_BYTES : bytesReceived;

    // If we got data, read it
    if (bytesReceived > 0)
    {
        static_cast<void>(m_pSerialPort->Read(m_SerialPortBuffer, bytesReceived));

        // See if its a packet intended for us
        if (memcmp(m_SerialPortBuffer, SERIAL_PORT_PACKET_HEADER, SERIAL_PORT_PACKET_HEADER_SIZE_BYTES) == 0)
        {
            // Next character is the command.  Array indexing starts at zero, thus no +1 on the size bytes constant
            int32_t command = static_cast<int32_t>(m_SerialPortBuffer[SERIAL_PORT_PACKET_HEADER_SIZE_BYTES]) - ASCII_0_OFFSET;

            // Sanity check it
            if (command >= 0 && command <= 9)
            {
                printf("Received a valid packet, command: %d\n", command);
            }
            else
            {
                printf("Invalid command received: %d\n", command);
            }
        }

        printf(m_SerialPortBuffer);
    }
    m_SerialPortBuffer[0] = NULL_CHARACTER;
*/
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::I2cSequence
///
/// This method contains the main workflow for interaction with
/// the I2C bus.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::I2cSequence()
{
    //uint8_t I2C_STRING[] = "i2c_roborio";
    std::memset(&m_I2cData, 0U, sizeof(m_I2cData));
    
    if (m_pI2cTimer->Get() > I2C_RUN_INTERVAL_S)
    {
        // Get the data from the riodiuino
        //static_cast<void>(m_pI2cPort->Transaction(I2C_STRING, sizeof(I2C_STRING), reinterpret_cast<uint8_t *>(&m_I2cData), sizeof(m_I2cData)));
        
        // Copy off the values for logic that relies on past distances
        static_cast<void>(std::memcpy(&m_RecentSonarValues[m_SonarDataIndex * I2cData::NUM_SONAR_SENSORS], &m_I2cData.m_FrontSonarA, I2cData::NUM_SONAR_SENSORS));
        
        // Look for wrap around
        m_SonarDataIndex++;
        if (m_SonarDataIndex > SONAR_DATA_INDEX_END)
        {
            m_SonarDataIndex = SONAR_DATA_INDEX_START;
        }
        
        if (DEBUG_PRINTS)
        {
            SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
            SmartDashboard::PutNumber("Left  Sonar A: ", m_I2cData.m_LeftSonarA);
            SmartDashboard::PutNumber("Left  Sonar B: ", m_I2cData.m_LeftSonarB);
            SmartDashboard::PutNumber("Back  Sonar A: ", m_I2cData.m_BackSonarA);
            SmartDashboard::PutNumber("Back  Sonar B: ", m_I2cData.m_BackSonarB);
            SmartDashboard::PutNumber("Right Sonar A: ", m_I2cData.m_RightSonarA);
            SmartDashboard::PutNumber("Right Sonar B: ", m_I2cData.m_RightSonarB);
        }
        
        if ((m_I2cData.m_FrontSonarA < SONAR_LED_WARN_DIST_INCHES) || (m_I2cData.m_FrontSonarB < SONAR_LED_WARN_DIST_INCHES))
        {
            m_bLed = true;
        }
        else
        {
            m_bLed = false;
        }
        
        // Restart the timer
        m_pI2cTimer->Reset();
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::CameraSequence
///
/// This method handles camera related behavior.  See the
/// RobotCamera class for full details.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::CameraSequence()
{    
    //cs::CvSink cvSink0 = CameraServer::GetInstance()->GetVideo(m_Cam0);
    //static_cast<void>(cvSink0.GrabFrame(m_Mat));
    //outputStream.PutFrame(m_Mat);
    
    if (m_pControlJoystick->GetRawButton(CAMERA_0_BUTTON))
    {
        m_CameraServer.SetSource(m_Cam0);
    }
    else if (m_pControlJoystick->GetRawButton(CAMERA_1_BUTTON))
    {
        m_CameraServer.SetSource(m_Cam1);
    }
    else
    {
    }
    
    // Now do the camera processing
    //bool bDoFullVisionProcessing = false;
    
    // To not kill the CPU/hog this thread, only do full
    // vision processing (particle analysis) periodically.
    if (m_pCameraRunTimer->Get() >= CAMERA_RUN_INTERVAL_S)
    {
        //bDoFullVisionProcessing = true;
        m_pCameraRunTimer->Reset();
    }
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::DriveControlSequence
///
/// This method contains the main workflow for drive control.
/// It will gather input from the drive joystick and then filter
/// those values to ensure they are past a certain threshold and
/// within range to send to the speed controllers.  Lastly it
/// will actually set the speed values.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::DriveControlSequence()
{
    // Computes what the maximum drive speed could be
    float throttleControl = GetThrottleControl(m_pDriveJoystick);

    // Get joystick inputs and make sure they clear a certain threshold.
    // This will help to drive straight.
    float xAxisDrive = Trim((m_pDriveJoystick->GetX() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);
    float yAxisDrive = Trim((m_pDriveJoystick->GetY() * throttleControl), JOYSTICK_TRIM_UPPER_LIMIT, JOYSTICK_TRIM_LOWER_LIMIT);

    // If the swap direction button was pressed, negate y value
    if ( m_bDriveSwap )
    {
        yAxisDrive *= -1;
    }

    // Filter motor speeds
    //float leftSpeed = Limit((xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    //float rightSpeed = Limit((xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    float leftSpeed = Limit((-xAxisDrive + yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    float rightSpeed = Limit((-xAxisDrive - yAxisDrive), DRIVE_MOTOR_UPPER_LIMIT, DRIVE_MOTOR_LOWER_LIMIT);
    
    // Set motor speed
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    /*
    // First check for inching controls
    if (m_pDriveJoystick->GetRawButton(INCH_FORWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, FORWARD);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_BACKWARD_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, REVERSE);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_LEFT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, LEFT);
    }
    else if (m_pDriveJoystick->GetRawButton(INCH_RIGHT_BUTTON))
    {
        DirectionalInch(INCHING_DRIVE_SPEED, RIGHT);
    }
    else
    {
        // Set motor speed
        m_pLeftDriveMotor->Set(leftSpeed);
        m_pRightDriveMotor->Set(rightSpeed);
    }
    */
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::DirectionalInch
///
/// This method contains the main workflow for drive directional
/// inching.  Based on input direction, it will briefly move the
/// robot a slight amount in that direction.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::DirectionalInch(float speed, EncoderDirection direction)
{
    // 2017 LEFT FORWARD DRIVE IS NEGATIVE
    // 2017 RIGHT FORWARD DRIVE IS POSITIVE
    float leftSpeed = speed;
    float rightSpeed = speed;
    
    // Negate appropriate motor speeds, based on direction
    switch (direction)
    {
        case FORWARD:
        {
            leftSpeed *= -1.0F;
            break;
        }
        case REVERSE:
        {
            rightSpeed *= -1.0F;
            break;
        }
        case LEFT:
        {
            break;
        }
        case RIGHT:
        {
            leftSpeed *= -1.0F;
            rightSpeed *= -1.0F;
            break;
        }
        default:
        {
            break;
        }
    }
    
    // Start the timer
    m_pInchingDriveTimer->Reset();
    m_pInchingDriveTimer->Start();
    
    // Motors on
    m_pLeftDriveMotor->Set(leftSpeed);
    m_pRightDriveMotor->Set(rightSpeed);
    
    while (m_pInchingDriveTimer->Get() < INCHING_DRIVE_DELAY_S)
    {
    }
    
    // Motors back off
    m_pLeftDriveMotor->Set(OFF);
    m_pRightDriveMotor->Set(OFF);
    
    // Stop the timer
    m_pInchingDriveTimer->Stop();
    m_pInchingDriveTimer->Reset();
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Test
///
/// This method is run when entering test mode.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::Test()
{
    while (true)
    {
    }
}



// EXECUTION START
START_ROBOT_CLASS(CmsdRobot);   // Macro to instantiate and run the class
