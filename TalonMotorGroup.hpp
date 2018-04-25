////////////////////////////////////////////////////////////////////////////////
/// @file TalonMotorGroup.hpp
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

#ifndef TALONMOTORGROUP_HPP
#define TALONMOTORGROUP_HPP

// SYSTEM INCLUDES
// <none>

// C INCLUDES
#include "CANTalon.h"
#include "CanTalonSRX.h"

// C++ INCLUDES
// (none)

class TalonMotorGroup
{
public:
    typedef CANTalon::FeedbackDevice FeedbackDevice;
    
	enum ControlMode
    {
        FOLLOW,
        INDEPENDENT,
        INVERSE,
        INDEPENDENT_OFFSET,
        INVERSE_OFFSET
    };

    // Constructor
    TalonMotorGroup( int numInstances, int firstCANId, ControlMode controlMode, FeedbackDevice sensor = static_cast<FeedbackDevice>(0) );
    
    // Function to set the speed of each motor in the group
    void Set( float value );
    void SetWithOffset( float group1Value, float group2Value );
    
    // Return the value of the sensor connected to the Talon
    int GetEncoderValue();
    void TareEncoder();
    
    // Change Talon mode between brake/coast
    void SetCoastMode();
    void SetBrakeMode();
    
private:
    static const int        MAX_NUMBER_OF_MOTORS    = 4;

    // Member variables
    int m_NumMotors;                                    // Number of motors in the group
    CANTalon *  m_pMotors[MAX_NUMBER_OF_MOTORS];        // The motor objects
    ControlMode m_ControlMode;                          // Keep track of the configuration of this Talon group
    FeedbackDevice m_Sensor;                            // Keep track of the sensor attached to the Talon
    
    static int m_DriveEncodersRelativeDiff;             // User controllable relative difference value of encoders for drive motors
    
    // Prevent default construction/deletion/copy/assignment
    TalonMotorGroup();
    ~TalonMotorGroup();
    TalonMotorGroup( const TalonMotorGroup& );
    TalonMotorGroup & operator=( const TalonMotorGroup& );
};

#endif // TALONMOTORGROUP_HPP
