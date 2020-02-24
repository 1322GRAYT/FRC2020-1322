/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.calibrations;

/**
 * Add your docs here.
 */
public class K_SWRV {



  /**************************************************/
  /*  Swerve Drive Configuration Calibrations       */
 	/**************************************************/	 	

	  /** KeSWRV_b_DebugEnbl: Swerve Drive System Enable
     *  Calibartion to send data to dashbord to debug..
     */
    public static final boolean KeSWRV_b_DebugEnbl = true;

    
	  /** KeSWRV_b_DrvMtrRotDirctnInvertInhb: Swerve Drive System Enable
     *  Calibartion to Inhibit Drive Motor Direction Inversion when 
     *  the Drive System is in Rotate Mode.
     */
    public static final boolean KeSWRV_b_DrvMtrRotDirctnInvertInhb = false;


  /**************************************************/
  /*  Swerve Drive General System Calibrations      */
 	/**************************************************/	 	

	  /** KeSWRV_r_CntlrDeadBandThrsh: Swerve Drive System: Normalized
     * Power Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized.  If the Absolute value of the Input
     * is below the Threshold it will be ignored (i.e. set to Zero).
     */
    public static final double KeSWRV_r_CntlrDeadBandThrsh = 0.2;


  /**************************************************/
  /*  Swerve Drive Design Parameters                */
 	/**************************************************/	 	

	  /** KeSWRV_r_RotMtrEncdrToCaddyRat: Swerve Drive System Number of
     * Rotation Control Motor Encoder Rotations to Swerve Module Rotations
     * Ratio.
     */
    public static final double KeSWRV_r_RotMtrEncdrToCaddyRat = 19.5;

	  /** KeSWRV_l_ChassisWhlBase: Swerve Drive System Drive Chassis
     * Effective WheelBase.
     */
    public static final double KeSWRV_l_ChassisWhlBase = 22.75;

	  /** KeSWRV_l_ChassisTrkWdth: Swerve Drive System Drive Chassis
     * Effective TrackWidtch.
     */
    public static final double KeSWRV_l_ChassisTrkWdth = 22;


	  /** KeSWRV_r_RotMtrEncdrToCaddyRat: Swerve Drive System - Conversion
     * Factor of the Number of Drive Motor Encoder Counts of Rotation
     * per 1 Linear inch of Drive Wheel travel.
     */
    public static final double KeSWRV_Cf_DrvMtrEncdrCntsToInch = 35.6;

    

  /****************************************************/
  /*  Swerve Drive Rotation Control PID Coefficients  */
 	/****************************************************/	 	
    
		// TODO: Tune! These are ripped directly off of REV's Example project

	  /** KeSWRV_K_RotProp: Swerve Drive System Rotation Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_RotProp = 0.1;


	  /** KeSWRV_K_RotPropIntgl: Swerve Drive System Rotation Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_RotIntgl = 1e-4;


	  /** KeSWRV_K_RotDeriv: Swerve Drive System Rotation Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_RotDeriv = 1;


	  /** KeSWRV_K_RotFdFwd: Swerve Drive System Rotation Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_RotFdFwd = 0;


	  /** KeSWRV_e_RotIntglErrMaxEnbl: Swerve Drive System Rotation Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). 
     */
    public static final double KeSWRV_e_RotIntglErrMaxEnbl = 0;


	  /** KeSWRV_r_RotNormOutMax: Swerve Drive System Rotation Control
     * Normalized Output Power Maximum Limit. 
     */
    public static final double KeSWRV_r_RotNormOutMax = 1;


	  /** KeSWRV_r_RotNormOutMin: Swerve Drive System Rotation Control
     * Normalized Output Power Minimum Limit. 
     */
    public static final double KeSWRV_r_RotNormOutMin = -1;
 


  /**************************************************/
  /*  Swerve Drive Drive Control PID Coefficients   */
 	/**************************************************/	 	
    
		// TODO: Tune! These are ripped directly off of REV's Example project

	  /** KeSWRV_K_DrvProp: Swerve Drive System Drive Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_DrvProp = 0.1;


	  /** KeSWRV_K_DrvPropIntgl: Swerve Drive System Drive Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_DrvIntgl = 1e-4;


	  /** KeSWRV_K_DrvDeriv: Swerve Drive System Drv Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_DrvDeriv = 1;


	  /** KeSWRV_K_DrvFdFwd: Swerve Drive System Drive Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_DrvFdFwd = 0;


	  /** KeSWRV_e_DrvIntglErrMaxEnbl: Swerve Drive System Drive Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). 
     */
    public static final int KeSWRV_e_DrvIntglErrMaxEnbl = 0;


	  /** KeSWRV_n_Drv_MM_CruiseVel: Swerve Drive System: This is the
     * peak target velocity that the motion magic curve generator can
     * use for the Drive Control Speed Closed-Loop. (RPM/100msec) 
     */
    public static final int KeSWRV_n_Drv_MM_CruiseVel = 9000;


	  /** KeSWRV_a_Drv_MM_MaxAccel: Swerve Drive System: Sets the Motion
     * Magic Acceleration. This is the target acceleration that the motion
     * magic curve generator can use for the Drive Control Speed
     * Closed-Loop.  (RPM/100msec/sec)
     */
    public static final int KeSWRV_a_Drv_MM_MaxAccel = 13000;


	  /** KeSWRV_r_DrvNormOutMax: Swerve Drive System Drive Control
     * Normalized Output Power Maximum Limit. 
     */
    public static final int KeSWRV_r_DrvNormOutMax = 1;


	  /** KeSWRV_r_DrvNormOutMin: Swerve Drive System Drive Control
     * Normalized Output Power Minimum Limit. 
     */
    public static final int KeSWRV_r_DrvNormOutMin = -1;
 



  /******************************************/
  /*  Swerve Drive Current Driver Limits    */
 	/******************************************/	 	
 
   /** KeSWRV_I_RotDrvrLmtMaxPri: Swerve Drive System Rotation Control
     * Motor Driver Maximum Current Limit for Primary Driver .
     */
    public static final int KeSWRV_I_RotDrvrLmtMaxPri = 60;

 
   /** KeSWRV_I_RotDrvrLmtMaxSec: Swerve Drive System Rotation Control
     * Motor Driver Maximum Current Limit for Secondary Driver. 
     */
    public static final double KeSWRV_I_RotDrvrLmtMaxSec = 20;

   /** KeSWRV_t_RotCAN_TmeOut: Swerve Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Milliseconds). 
     */
    public static final int KeSWRV_t_RotCAN_TmeOut = 0;



   /** KeSWRV_I_DrvDrvrLmtMaxPri: Swerve Drive System: Drive Control
     * Motor Driver Maximum Current Limit for Primary Driver,
     * Supply Current Limit.
     */
    public static final double KeSWRV_I_DrvDrvrLmtMaxPri = 15;

 
   /** KeSWRV_I_DrvDrvrLmtMaxSec: Swerve Drive System Drive Control
     * Motor Driver Maximum Current Limit for Secondary Driver,
     * Stator Current Limit. 
     */
    public static final double KeSWRV_I_DrvDrvrLmtMaxSec = 15;


   /** KeSWRV_t_DrvCAN_TmeOut: Swerve Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Milliseconds). 
     */
    public static final int KeSWRV_t_DrvCAN_TmeOut = 0;



}
