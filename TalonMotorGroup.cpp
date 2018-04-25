////////////////////////////////////////////////////////////////////////////////
/// @file TalonMotorGroup.cpp
///
/// A class designed to create a group of CAN Talons working in tandem.
///
/// CMSD FRC 2017
/// Author: David Stalter
/// @Edit History
/// - dts   03-JAN-2015 Created from 2014.
/// - dts   17-JAN-2015 Ported to CAN Talons.
/// - dts   06-FEB-2015 Support for follow and inverse control.
/// - dts   08-JAN-2017 Ported to use TalonSRX class.
///
////////////////////////////////////////////////////////////////////////////////

// SYSTEM INCLUDES
// <none>

// C INCLUDES
// (none)

// C++ INCLUDES
#include "TalonMotorGroup.hpp"      // For class declaration

// STATIC MEMBER DATA
int TalonMotorGroup::m_DriveEncodersRelativeDiff = 0;



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::TalonMotorGroup
///
/// Constructor.  Creates the number of motors specified on the
/// port numbers passed in.
///
////////////////////////////////////////////////////////////////
TalonMotorGroup::TalonMotorGroup( int numInstances, int firstCANId, ControlMode controlMode, FeedbackDevice sensor )
: m_NumMotors(numInstances)
, m_pMotors()
, m_ControlMode(controlMode)
, m_Sensor(sensor)
{
    // Allocate the necessary storage for all of the objects by invoking operator new[]
    // Assign the returned memory block to the first pointer in the array
    //m_pMotors[0] =  reinterpret_cast<CANTalon *>( operator new[] (numInstances * sizeof(CANTalon)) );

    // CAN Talons can be set to follow, which the motor groups
    // may do, so save off the first id as the master
    int masterId = firstCANId;

    // Loop for each motor to create
    for ( int i = 0; i < numInstances; i++ )
    {
        // Create it
        m_pMotors[i] = new CANTalon(firstCANId++);
        
        // Override to always coast
        m_pMotors[i]->ConfigNeutralMode(CANTalon::kNeutralMode_Coast);

        // Only set follow for Talon groups that will be configured
        // as such.  Otherwise just use the defaults (percent voltage based).
        if ((i != 0) && (controlMode == FOLLOW))
        {
            m_pMotors[i]->SetTalonControlMode(CANTalon::kFollowerMode);
            m_pMotors[i]->Set(masterId);
        }
        else
        {
            m_pMotors[i]->SetTalonControlMode(CANTalon::kThrottleMode);
            
            // Sensor initialization
            //m_pMotors[i]->SetFeedbackDevice(CANTalon::CtreMagEncoder_Relative);
            m_pMotors[i]->SetFeedbackDevice(sensor);
        }
    }
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::TareEncoder
///
/// Method to tare the value on an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::TareEncoder()
{
    if (m_Sensor == FeedbackDevice::CtreMagEncoder_Relative)
    {
        m_pMotors[0]->SetEncPosition(0);
    }
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::SetCoastMode
///
/// Method to change a talon to coast mode.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetCoastMode()
{
    m_pMotors[0]->ConfigNeutralMode(CANTalon::kNeutralMode_Coast);
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::SetBrakeMode
///
/// Method to change a talon to brake mode.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetBrakeMode()
{
    m_pMotors[0]->ConfigNeutralMode(CANTalon::kNeutralMode_Brake);
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::GetEncoderValue
///
/// Method to get the value from an encoder feedback device
/// connected to a Talon controller.
///
////////////////////////////////////////////////////////////////
int TalonMotorGroup::GetEncoderValue()
{
    if (m_Sensor == FeedbackDevice::CtreMagEncoder_Relative)
    {
        return m_pMotors[0]->GetEncPosition();
    }
    else
    {
        return 0;
    }
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::Set
///
/// Method to set the speed of each motor in the group.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::Set( float value )
{
    // Check what kind of group this is.  Most
    // CAN Talons will be set to follow, but some
    // may be independent or inverse (such as if
    // they need to drive in different directions).
    switch (m_ControlMode)
    {
        // Typical case, just update the master
        case FOLLOW:
        {
            m_pMotors[0]->Set(value);
            break;
        }
        case INDEPENDENT:
        {
            for (int i = 0; i < m_NumMotors; i++)
            {
                m_pMotors[i]->Set(value);
            }
            break;
        }
        // Motors are attached to drive in
        // opposite directions
        case INVERSE:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
               m_pMotors[i]->Set(value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                   m_pMotors[i]->Set(-value);
            }
            break;
        }
        // Default cases for the offsets
        case INDEPENDENT_OFFSET:
        case INVERSE_OFFSET:
        {
            SetWithOffset(value, value);
            break;
        }
        default:
        {
            break;
        }
    };
}



////////////////////////////////////////////////////////////////
// @method TalonMotorGroup::SetSpeed
///
/// Method to set the speed of each motor in the group, where
/// the speed is different between motors in the group.
///
////////////////////////////////////////////////////////////////
void TalonMotorGroup::SetWithOffset( float group1Value, float group2Value )
{
    // Check what kind of group this is.  This Talon
    // group is not uniform, so different values need
    // to be applied.
    switch (m_ControlMode)
    {
        case INDEPENDENT_OFFSET:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
               m_pMotors[i]->Set(group1Value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                   m_pMotors[i]->Set(group2Value);
            }
            break;
        }
        // Motors are attached to drive in
        // opposite directions
        case INVERSE_OFFSET:
        {
            // Assumes each half of motors need to go the same direction
            // (i.e. 1:n motors, 1:n/2 forward, n/2:n reverse
            for (int i = 0; i < m_NumMotors / 2; i++)
            {
               m_pMotors[i]->Set(group1Value);
            }
            for (int i = m_NumMotors / 2; i < m_NumMotors; i++)
            {
                   m_pMotors[i]->Set(-group2Value);
            }
            break;
        }
        default:
        {
            break;
        }
    };
}
