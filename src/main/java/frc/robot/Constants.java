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
        GAIN_POS_CONTROL, GAIN_ROT_CONTROL
    }
    /* MOTOR CAN ADDRESS ASSIGNMENTS */
    // NOTE: These are tenitive and are subject to change
    // Caddy Rotation Motors (SPARK MAX)
    public static final int FRONT_RIGHT_DRIVE_ROTATE = 6;
    public static final int REAR_RIGHT_DRIVE_ROTATE  = 1;
    public static final int FRONT_LEFT_DRIVE_ROTATE  = 2;
    public static final int REAR_LEFT_DRIVE_ROTATE   = 3;
    // Drive Motors (TALON FX)
    public static final int FRONT_RIGHT_DRIVE_DRIVE = 4;
    public static final int REAR_RIGHT_DRIVE_DRIVE  = 1;
    public static final int FRONT_LEFT_DRIVE_DRIVE  = 2;
    public static final int REAR_LEFT_DRIVE_DRIVE   = 3;
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
    // Wheel Spinner (CAN TALON SRX)
    public static final int COLOR_WHEEL_SPINNER     = 10;
    
    /* Digital Inputs */
    public static final int BALL_SENSE_INPUT = 1;
    public static final int BALL_SENSE_OUTPUT = 2;    

    /* Air Things */
    public static final int HANG_BAR_0 = 0;
    public static final int HANG_BAR_1 = 1;
    public static final int DRIVE_SHIFT_0 = 2;
    public static final int DRIVE_SHIFT_1 = 3;
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
    public static final double COLOR_WHEEL_SPIN_SPEED = .5;

    /* FOR INTAKE RAISE/LOWER COMMAND */
    public static final double RAISE_INTAKE_TIME_SEC = .5;
    public static final double LOWER_INTAKE_TIME_SEC = .5;
    public static final double RAISE_LOWER_INTAKE_SPEED = 1;
}
