/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.ColorMatch;

import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    // Enum for Solenoid control
    public enum SolenoidPosition {
        UP, DOWN, OFF
    }
    // Enum for Color Wheel Command
    public enum ColorWheelCommandExecute {
        GAIN_POS_CONTROL, GAIN_ROT_CONTROL, GAIN_BOTH_SAME_TIME, STOP_ALL
    }
    // Enum for Colors
    // .toString() returns the name, so RED.toString() returns "RED"
    public enum ColorWheelColor { 
        RED, GREEN, BLUE, YELLOW, UNKNOWN
    } 
    /* MOTOR CAN ADDRESS ASSIGNMENTS */
    // NOTE: These are tentative and are subject to change
    // Drive Motors (TALON FX)
    public static final int DRV_LT_FR               = 3;  //*  Front Left
    public static final int DRV_LT_RR               = 2;  //*  Rear  Left   (Correct)
    public static final int DRV_RT_FR               = 4;  //*  Front Right  (Inverted)
    public static final int DRV_RT_RR               = 1;  //*  Rear Right   (Inverted)
    // Shooter Motors (SPARK MAX)
    public static final int SHOOTER_ONE             = 4;
    public static final int SHOOTER_TWO             = 5;
    // Robot Lift (SPARK MAX)
    public static final int ROBOT_LIFT              = 7;
    // Shooter Tilt (CAN TALON SRX)
    public static final int SHOOTER_AIM_TILT        = 5;
    // Shooter PAN (CAN TALON SRX)
    public static final int SHOOTER_AIM_PAN         = 6;
    // Shooter Intake (CAN TALON SRX)
    public static final int SHOOTER_BALL_INTAKE     = 7;
    // Shooter Intake Lift (CAN TALON SRX)
    public static final int SHOOTER_INTAKE_LIFT     = 8;
    // Shooter Advance (CAN TALON SRX)
    public static final int SHOOTER_BALL_ADVANCE    = 9;
    // Wheel Spinner (CAN Victor SPX)
    public static final int COLOR_WHEEL_SPINNER     = 10;
    
    /* Digital Inputs */
    public static final int BALL_SENSE_INPUT  = 1;
    public static final int BALL_SENSE_OUTPUT = 2;    
    public static final int SWRV_ZERO_FR_RT   = 4;
    public static final int SWRV_ZERO_FR_LT   = 6;
    public static final int SWRV_ZERO_RR_LT   = 5;
    public static final int SWRV_ZERO_RR_RT   = 3;

    /* Air Things */
    public static final int HANG_BAR_0 = 0;
    public static final int HANG_BAR_1 = 1;
    public static final int COLOR_WHEEL_0 = 4;
    public static final int COLOR_WHEEL_1 = 5;

    
    /* COLOR CALIBRATIONS FOR COLOR WHEEL */
    // TODO: Calibrate at each event
    public static final Color kBlueTarget = ColorMatch.makeColor(0.143, 0.427, 0.429);
    public static final Color kGreenTarget = ColorMatch.makeColor(0.197, 0.561, 0.240);
    public static final Color kRedTarget = ColorMatch.makeColor(0.385, 0.436, 0.209);
    public static final Color kYellowTarget = ColorMatch.makeColor(0.361, 0.524, 0.113);

    /* SPEED TO SPIN COLOR WHEEL AT */
    // Remember, we SHOULD NOT spin the wheel faster than 60 RPM
    // This is a CANTalon Speed, so -1 - 0 - 1
    public static final double COLOR_WHEEL_SPIN_SPEED = .25;
    /* IDEAL PROXIMITY OF COLOR WHEEL */
    public static final int    IDEAL_PROX_COLOR_SENSOR = 310;
    public static final int    PROX_SENSOR_TOLERANCE = 50;

    /* FOR INTAKE RAISE/LOWER COMMAND */
    public static final double RAISE_INTAKE_TIME_SEC = .5;
    public static final double LOWER_INTAKE_TIME_SEC = .5;
    public static final double LOWER_INTAKE_SPEED = .5;
    public static final double RAISE_INTAKE_SPEED = 1;


    /* X-BOX CONTROLLER MAPPING */
    // Controller Assignments
    public static final int DRVR_CNTRLR       =  0;
    public static final int AUX_CNTRLR        =  1;
    // Button Assignments
    public static final int BUTTON_A          =  1;
    public static final int BUTTON_B          =  2;
    public static final int BUTTON_X          =  3;
    public static final int BUTTON_Y          =  4;
    public static final int BUMPER_LEFT       =  5;
    public static final int BUMPER_RIGHT      =  6;
    public static final int BUTTON_BACK       =  7;  // LEFT(SELECT)
    public static final int BUTTON_START      =  8;  // RIGHT
    public static final int STICK_LEFT_PRESS  =  9;  // JOYSTICK PRESS
    public static final int STICK_RIGHT_PRESS = 10;  // JOYSTICK PRESS
    // Analog Assignments
    public static final int STICK_LEFT_XAXIS  = 1;
    public static final int STICK_LEFT_YAXIS  = 2;
    public static final int TRIGGERS          = 3;
    public static final int STICK_RIGHT_XAXIS = 4;
    public static final int STICK_RIGHT_YAXIS = 5;
    public static final int DPAD              = 6;


}
