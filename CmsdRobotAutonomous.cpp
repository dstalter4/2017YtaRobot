////////////////////////////////////////////////////////////////////////////////
/// @file CmsdRobotAutonomous.cpp
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



// Field Layout (Measurements from Palmetto)
//      72"    72"  32"  72"
//    ______|______|___|______
//   /|
//  / |  36"
// /__|
// | 36" (This is a 45-45-90 triangle)
// |
// |
// - 96" (from wall)
// |
// |------------------> Airship 132" (roughly)
// - 132" (from wall, roughly)

// Row-bit
//
// -----      -----
// | ^            |
// | |            |
// | 28"          |
// | |            |
// | v            |
// ----------------
//    <-- 32" -->
// Bumpers = 3.5"

// => Row-bit has x = 39", y = 35"

// Traversal Distances:
// 1. y-axis driver station move (align edge of alliance wall plate): 0", 72", 176"
// 2. x-axis to fuel station move: 120" - 35" = 85"
// 3. y-axis to fuel station move: 36"
// 4. y-axis back away from fuel station move: 12"



////////////////////////////////////////////////////////////////
// @method CmsdRobot::Autonomous
///
/// The autonomous control method.  This method is called once
/// each time the robot enters autonomous control.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::Autonomous()
{
    // Put everything in a stable state
    InitialStateSetup();
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
    m_pSafetyTimer->Stop();
    m_pSafetyTimer->Reset();
    
    // Autonomous will use brake mode
    m_pLeftDriveMotor->SetBrakeMode();
    m_pRightDriveMotor->SetBrakeMode();
    
    // Change values in the header to control having an
    // autonomous routine and which is selected
    
    //const bool bAuto1 = m_pAutonomous1Switch->Get();
    //const bool bAuto2 = m_pAutonomous2Switch->Get();

    // Auto routine 1
    //if ( CmsdRobotAutonomous::ROUTINE_1 )
    if ( m_pAutonomous1Switch->Get() )
    //if ( bAuto1 && !bAuto2 )
    {
        AutonomousRoutine1();
        while ( m_pDriverStation->IsAutonomous()) {}
    }
    
    // Auto routine 2
    //else if ( CmsdRobotAutonomous::ROUTINE_2 )
    else if ( m_pAutonomous2Switch->Get() )
    //else if ( !bAuto1 && bAuto2 )
    {
        AutonomousRoutine2(true);
        while ( m_pDriverStation->IsAutonomous() ) {}
    }
    
    // Auto routine 3
    //else if ( CmsdRobotAutonomous::ROUTINE_3 )
    else if ( m_pAutonomous3Switch->Get() )
    //else if ( bAuto1 && bAuto2 )
    {
        AutonomousRoutine3();
        while ( m_pDriverStation->IsAutonomous() ) {}
    }

    /* !!! ONLY ENABLE TEST AUTONOMOUS CODE WHEN TESTING
           SELECT A FUNCTIONING ROUTINE FOR ACTUAL MATCHES !!! */
    else if ( CmsdRobotAutonomous::TEST_ENABLED )
    {
        // This code will never return
        AutonomousTestCode();
    }

    else
    {
        // No option was selected; ensure known behavior to avoid issues
        while ( m_pDriverStation->IsAutonomous() ) {}
    }

}   // End Autonomous



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousShoot
///
/// Shoot fuel balls during autonomous.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousShoot()
{
    //static int shotNumber = 0;
    
    
    // Turn injector and shooter motors on
    //m_pFuelPumpMotor->Set(CmsdRobotAutonomous::FUEL_PUMP_MOTOR_SPEED);
    m_pFuelShooterMotor->Set(-CmsdRobotAutonomous::FUEL_SHOOT_MOTOR_SPEED);
    AutonomousDelay(2.0F);
    m_pFuelInjectorMotor->Set(CmsdRobotAutonomous::FUEL_INJECT_MOTOR_SPEED);

    m_pAgitatorMotor->Set(-ON * AGITATOR_SCALING);
    
    //bool bPulseInProgress = false;
    bool bAgitatorOn = true;
    Timer agitatorTimer;
    agitatorTimer.Reset();
    agitatorTimer.Start();
    
    // Timed shot
    m_pAutonomousTimer->Start();
    while (m_pAutonomousTimer->Get() < CmsdRobotAutonomous::SHOOT_DELAY_S)
    {
        if (agitatorTimer.Get() >= 0.50F)
        {
            if (bAgitatorOn)
            {
                m_pAgitatorMotor->Set(OFF);
            }
            else
            {
                m_pAgitatorMotor->Set(-ON * AGITATOR_SCALING);
            }
            bAgitatorOn = !bAgitatorOn;
            agitatorTimer.Reset();
        }
        
        if (!m_pDriverStation->IsAutonomous())
        {
            break;
        }
    }
    agitatorTimer.Stop();
    agitatorTimer.Reset();
    /*
        if (!bPulseInProgress)
        {
            //m_pFuelPumpMotor->TareEncoder();
            //m_pFuelPumpMotor->Set(CmsdRobotAutonomous::FUEL_PUMP_MOTOR_SPEED);
            bPulseInProgress = true;
            shotNumber++;
        }
        else
        {
            //if (m_pFuelPumpMotor->GetEncoderValue() >= ENCODER_TURNS_PER_FUEL_PUMP_PULSE)
            if (true)
            {
                //m_pFuelPumpMotor->Set(-ON * MOTOR_BACK_DRIVE_SPEED);
                m_pFuelPumpTimer->Reset();
                m_pFuelPumpTimer->Start();
                
                // Tare the encoder so that we don't come back in here
                //m_pFuelPumpMotor->TareEncoder();
            }
        
            if (m_pFuelPumpTimer->Get() >= FUEL_PUMP_SPACING_DELAY_S)
            {
                //m_pFuelPumpMotor->Set(OFF);
                
                m_pFuelPumpTimer->Stop();
                m_pFuelPumpTimer->Reset();
                
                // Every other shot, back drive to prevent jamming
                if ((shotNumber % 3) == 0)
                {
                    //m_pFuelPumpMotor->Set(-ON * FUEL_PUMP_SCALING_REVERSE);
                    m_pFuelPumpTimer->Start();
                    while (m_pFuelPumpTimer->Get() < FUEL_PUMP_SPACING_DELAY_S)
                    {
                    }
                    //m_pFuelPumpMotor->Set(OFF);
                    m_pFuelPumpTimer->Stop();
                    m_pFuelPumpTimer->Reset();
                }
                bPulseInProgress = false;
            }
        }
    }
    */
    m_pAutonomousTimer->Stop();
    m_pAutonomousTimer->Reset();
    
    // Motors off
    //m_pFuelPumpMotor->Set(OFF);
    m_pFuelInjectorMotor->Set(OFF);
    m_pFuelShooterMotor->Set(OFF);
    m_pAgitatorMotor->Set(OFF);
    
    AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
}



////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousCommon
///
/// Common autonomous behavior.  It moves away from the alliance
/// wall and to the fuel loading station.  The variance is
/// whether it shoots at the start or at the end.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousCommon(bool bStartShoot, bool bEndShoot)
{

////////////////////////////////////////////////////////////////////////////////
// Initial state setup based on alliance.
////////////////////////////////////////////////////////////////////////////////
    // Turn the agitator on
    m_pAgitatorMotor->Set(ON);
    if (bStartShoot)
    {
        AutonomousShoot();
    }
    
    if (m_AllianceColor == Alliance::kRed)
    {
        AutonomousCommonRed(-CmsdRobotAutonomous::DRIVE_SPEED_SLOW, bEndShoot);
    }
    else if (m_AllianceColor == Alliance::kBlue)
    {
        AutonomousCommonBlue(CmsdRobotAutonomous::DRIVE_SPEED_SLOW, bEndShoot);
    }
    else
    {
    }
    
    m_pAgitatorMotor->Set(OFF);
}





////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousCommonRed
///
/// Common autonomous behavior when on the red alliance.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousCommonRed(float driveSpeed, bool bEndShoot)
{
    volatile bool bIsAutonomous = true;
////////////////////////////////////////////////////////////////////////////////
// Turn away from the wall.
////////////////////////////////////////////////////////////////////////////////
    //bIsAutonomous = m_pDriverStation->IsAutonomous();
    //if (bIsAutonomous)
    //{
        // Gyro turn is protected by the safety timer
        //bool bResult = AutonomousGyroLeftTurn((CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE + CmsdRobotAutonomous::TURN_ANGLE_SLOP_DEGREES), CmsdRobotAutonomous::TURN_SPEED);
        //AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
        
        //if (!bResult)
        //{
        //  return;
        //}
    //}
    //else
    //{
        //return;
    //}
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Back away from the alliance wall.
////////////////////////////////////////////////////////////////////////////////
    // Starting a new drive operation, tare encoders
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            AutonomousEncoderDrive(driveSpeed, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        }
        
        // |
        // |    S              S
        // ||3.5|     28.5     |3.5|     -->          120
        //      | <---   32   ---> |     -->          (120 - 32 = 88)
        //
        // TODO: Add increase/decrease continuity checks and repeated out of range readings before failing.
        //while ((m_I2cData.m_FrontSonarA < CmsdRobotAutonomous::SONAR_LATERAL_DRIVE_DIST_INCHES) && (m_I2cData.m_FrontSonarB < CmsdRobotAutonomous::SONAR_LATERAL_DRIVE_DIST_INCHES));
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -28000.0F) && (m_pRightDriveMotor->GetEncoderValue() > -28000.0F));
        
        // Negative, this drive is backwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() > -CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Turn towards fuel loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        AutonomousGyroLeftTurn(CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE + CmsdRobotAutonomous::TURN_ANGLE_SLOP_DEGREES + CmsdRobotAutonomous::TURN_ANGLE_EXTRA_SLOP_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    else
    {
        return;
    }
        
    
    
////////////////////////////////////////////////////////////////////////////////
// Drive towards fuel loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            // Negative because it is the opposite direction of
            // moving away from the alliance wall.
            AutonomousEncoderDrive(-driveSpeed * 2.0F, 0.0F, FORWARD);
            //AutonomousEncoderDrive(-driveSpeed - CmsdRobotAutonomous::DRIVE_RAMMING_SPEED, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        }
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) < 15000.0F) && (m_pRightDriveMotor->GetEncoderValue() < 15000.0F));
        
        // Positive, this drive is forwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) < CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() < CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
        
        // Delay to gather some balls
        AutonomousDelay(CmsdRobotAutonomous::GATHER_FUEL_BALLS_DELAY_S);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Back up from loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            AutonomousEncoderDrive(driveSpeed, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        }
        // Negative, this drive is backwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -4000.0F) && (m_pRightDriveMotor->GetEncoderValue() > -4000.0F));
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() > -CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
        
        AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Turn towards the goal.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        if (!AutonomousGyroRightTurn(CmsdRobotAutonomous::TURN_TO_BOILER_LOAD_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED))
        {
            return;
        }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Shoot.
////////////////////////////////////////////////////////////////////////////////
        if (bEndShoot)
        {
            AutonomousShoot();
        }
    }
}





////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
///                           Red/Blue Separation                            ///
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////





////////////////////////////////////////////////////////////////
// @method CmsdRobot::AutonomousCommonBlue
///
/// Common autonomous behavior when on the blue alliance.
///
////////////////////////////////////////////////////////////////
void CmsdRobot::AutonomousCommonBlue(float driveSpeed, bool bEndShoot)
{
    volatile bool bIsAutonomous = true;
////////////////////////////////////////////////////////////////////////////////
// Turn away from the wall.
////////////////////////////////////////////////////////////////////////////////
    //bIsAutonomous = m_pDriverStation->IsAutonomous();
    //if (bIsAutonomous)
    //{
        // Gyro turn is protected by the safety timer
        //bool bRestul = AutonomousGyroRightTurn((CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE + CmsdRobotAutonomous::TURN_ANGLE_SLOP_DEGREES), CmsdRobotAutonomous::TURN_SPEED);
        //AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
        //if (!bResult)
        //{
        //  return;
        //}
    //}
    //else
    //{
        //return;
    //}
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Back away from the alliance wall.
////////////////////////////////////////////////////////////////////////////////
    // Starting a new drive operation, tare encoders
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            //AutonomousEncoderDrive(-CmsdRobotAutonomous::DRIVE_SPEED_SLOW*1.5F, 0.0F, FORWARD);
            AutonomousEncoderDrive(driveSpeed, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        }
        
        // |
        // |    S              S
        // ||3.5|     28.5     |3.5|     -->          120
        //      | <---   32   ---> |     -->          (120 - 32 = 88)
        //
        // TODO: Add increase/decrease continuity checks and repeated out of range readings before failing.
        //while ((m_I2cData.m_FrontSonarA < CmsdRobotAutonomous::SONAR_LATERAL_DRIVE_DIST_INCHES) && (m_I2cData.m_FrontSonarB < CmsdRobotAutonomous::SONAR_LATERAL_DRIVE_DIST_INCHES));
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) < 28000.0F) && (m_pRightDriveMotor->GetEncoderValue() < 28000.0F));
        
        // Positive, this drive is forwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) < CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() < CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_FIRST_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Turn towards fuel loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        AutonomousGyroRightTurn(CmsdRobotAutonomous::NINETY_DEGREE_TURN_ANGLE + CmsdRobotAutonomous::TURN_ANGLE_SLOP_DEGREES + CmsdRobotAutonomous::TURN_ANGLE_EXTRA_SLOP_DEGREES, CmsdRobotAutonomous::TURN_SPEED);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Drive towards fuel loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            // Negative because it is the opposite direction of
            // moving away from the alliance wall.
            AutonomousEncoderDrive(-driveSpeed * 2.0F, 0.0F, FORWARD);
            //AutonomousEncoderDrive(driveSpeed + CmsdRobotAutonomous::DRIVE_RAMMING_SPEED, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        
        }
        
        // Negative, this drive is backwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() > -CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() > -GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_SECOND_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
        
        // Delay to gather some balls
        AutonomousDelay(CmsdRobotAutonomous::GATHER_FUEL_BALLS_DELAY_S);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Back up from loading station.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        m_pLeftDriveMotor->TareEncoder();
        m_pRightDriveMotor->TareEncoder();
        m_pSafetyTimer->Reset();
        m_pSafetyTimer->Start();
        do
        {
            //I2cSequence();
            
            AutonomousEncoderDrive(driveSpeed, 0.0F, FORWARD);
            
            //SmartDashboard::PutNumber("Front Sonar A: ", m_I2cData.m_FrontSonarA);
            //SmartDashboard::PutNumber("Front Sonar B: ", m_I2cData.m_FrontSonarB);
        }
        
        // Positive, this drive is forwards
        //while ((-(m_pLeftDriveMotor->GetEncoderValue()) < CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_TURNS) && (m_pRightDriveMotor->GetEncoderValue() < CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_TURNS));
        while ((-(m_pLeftDriveMotor->GetEncoderValue()) < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES)) 
              && (m_pRightDriveMotor->GetEncoderValue() < GetEncoderRotationsFromInches(CmsdRobotAutonomous::ENCODER_DRIVE_THIRD_DIST_IN, DRIVE_WHEEL_DIAMETER_INCHES))
              && (m_pSafetyTimer->Get() <= SAFETY_TIMER_MAX_VALUE));
        m_pSafetyTimer->Stop();
        
        AutonomousBackDrive(FORWARD);
        
        AutonomousDelay(CmsdRobotAutonomous::DELAY_SHORT_S);
    }
    else
    {
        return;
    }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Turn towards the goal.
////////////////////////////////////////////////////////////////////////////////
    bIsAutonomous = m_pDriverStation->IsAutonomous();
    if (bIsAutonomous)
    {
        if (!AutonomousGyroLeftTurn(CmsdRobotAutonomous::TURN_TO_BOILER_LOAD_ANGLE_DEGREES, CmsdRobotAutonomous::TURN_SPEED))
        {
            return;
        }
    
    
    
////////////////////////////////////////////////////////////////////////////////
// Shoot.
////////////////////////////////////////////////////////////////////////////////
        if (bEndShoot)
        {
            AutonomousShoot();
        }
    }
}
