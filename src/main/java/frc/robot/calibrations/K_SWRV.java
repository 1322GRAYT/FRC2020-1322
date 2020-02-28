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
     *  Calibartion to send data to dashbord to debug.
     */
    public static final boolean KeSWRV_b_DebugEnbl = true;

    
	  /** KeSWRV_b_DrvMtrRotDirctnInvertEnbl: Swerve Drive System Enable
     *  Calibration to Enable Drive Motor Direction Inversion.
     */
    public static final boolean KeSWRV_b_DrvMtrRotDirctnInvertEnbl = false;

	  /** KeSWRV_b_DrvMtrRotDirctnInvertInhbRot: Swerve Drive System Enable
     *  Calibration to Inhibit Drive Motor Direction Inversion when 
     *  the Drive System is in Rotate Mode.
     */
    public static final boolean KeSWRV_b_DrvMtrRotDirctnInvertInhbRot = true;



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

	  /** KeSWRV_l_ChassisWhlBase: Swerve Drive System Drive Chassis
     * Effective WheelBase.
     */
    public static final double KeSWRV_l_ChassisWhlBase = 26.34;

	  /** KeSWRV_l_ChassisTrkWdth: Swerve Drive System Drive Chassis
     * Effective TrackWidtch.
     */
    public static final double KeSWRV_l_ChassisTrkWdth = 17.25;



  /**************************************************/
  /*  Swerve Rotation-System Mechanical Parameters  */
 	/**************************************************/	 	

	  /** KeSWRV_Cf_RotMtrEncdrCntsPerRev: Swerve Drive System - Conversion
     * Factor of the Number of Encoder Counts Per One Rotation of the
     * Rotation Control Motor.
     */
    public static final double KeSWRV_Cf_RotMtrEncdrCntsPerRev = 42;


	  /** KeSWRV_r_RotMtrEncdrToCaddyRat: Swerve Drive System: Number of
     * Rotation Control Motor Encoder Rotations to Swerve Module Rotations
     * Ratio.
     */
    public static final double KeSWRV_r_RotMtrEncdrToCaddyRat = 19.5;



  /*******************************************************/
  /*  Swerve Drive-System Design Mechanical Parameters   */
 	/*******************************************************/	 	
	
	  /** KeSWRV_Cf_DrvEncdrCntsPerInchHi: Swerve Drive System - Conversion
     * Factor of the Number of Drive Motor Encoder Counts of Rotation
     * per 1 Linear inch of Drive Wheel travel in Low Gear Ratio.
     */
    public static final double KeSWRV_Cf_DrvEncdrCntsPerInchHi = 35.6;


	  /** KeSWRV_Cf_DrvEncdrCntsPerInchLo: Swerve Drive System - Conversion
     * Factor of the Number of Drive Motor Encoder Counts of Rotation
     * per 1 Linear inch of Drive Wheel travel in Low Gear Ratio.
     */
    public static final double KeSWRV_Cf_DrvEncdrCntsPerInchLo = 35.6;


	  /** KeSWRV_Cnt_DrvEncdrCntsPerRev: Swerve Drive System: Number
     * of Encoder Counts Per One Revolution of the Drive Motors
     */
    public static final double KeSWRV_Cnt_DrvEncdrCntsPerRev = 360;


	  /** KeSWRV_l_DrvWhlDistPerRot: Swerve Drive System: Number of lineal
     * distance travaled per one rotation of Swerve Drive Wheel Rotation
     */
    public static final double KeSWRV_l_DrvWhlDistPerRot = 4.0 * Math.PI; // 12.57142857


    /** KeSWRV_r_DrvMtrEncdrToWhlRatLo: Swerve Drive System: Number of
     * Drive Control Motor Encoder Rotations to Swerve Wheel Rotations
     * Ratio for Low Gear Ration.
     */
    public static final double KeSWRV_r_DrvMtrEncdrToWhlRatLo = 11.87016;


	  /** KeSWRV_r_DrvMtrEncdrToWhlRatHi: Swerve Drive System: Number of
     * Drive Control Motor Encoder Rotations to Swerve Wheel Rotations
     * Ratio for High Gear Ratio.
     */
    public static final double KeSWRV_r_DrvMtrEncdrToWhlRatHi = 8.166667;



  /****************************************************/
  /*  Swerve Drive Rotation Control PID Coefficients  */
 	/****************************************************/	 	
    
	  /** KeSWRV_K_RotProp: Swerve Drive System Rotation Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_RotProp = 2.0;


	  /** KeSWRV_K_RotPropIntgl: Swerve Drive System Rotation Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_RotIntgl = 0.001;


	  /** KeSWRV_K_RotDeriv: Swerve Drive System Rotation Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_RotDeriv = 0.0;


	  /** KeSWRV_K_RotFdFwd: Swerve Drive System Rotation Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_RotFdFwd = 0.0;


	  /** KeSWRV_r_RotIntglErrMaxEnbl: Swerve Drive System Rotation Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). (revs)
     */
    public static final double KeSWRV_r_RotIntglErrMaxEnbl = 0.1;


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

	  /** KeSWRV_K_DrvProp: Swerve Drive System Drive Control
     * Proporational Control Gain. 
     */
    public static final double KeSWRV_K_DrvProp = 2.0;


	  /** KeSWRV_K_DrvPropIntgl: Swerve Drive System Drive Control
     * Integral Control Gain. 
     */
    public static final double KeSWRV_K_DrvIntgl = 0.0;


	  /** KeSWRV_K_DrvDeriv: Swerve Drive System Drv Control
     * Derivative Control Gain. 
     */
    public static final double KeSWRV_K_DrvDeriv = 0.0;


	  /** KeSWRV_K_DrvFdFwd: Swerve Drive System Drive Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeSWRV_K_DrvFdFwd = 0.0;


	  /** KeSWRV_n_DrvIntglErrMaxEnbl: Swerve Drive System Drive Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). (rpm/100ms) 
     */
    public static final int KeSWRV_n_DrvIntglErrMaxEnbl = 0;


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
    public static final double KeSWRV_I_DrvDrvrLmtMaxPri = 40;

 
   /** KeSWRV_I_DrvDrvrLmtMaxSec: Swerve Drive System Drive Control
     * Motor Driver Maximum Current Limit for Secondary Driver,
     * Stator Current Limit. 
     */
    public static final double KeSWRV_I_DrvDrvrLmtMaxSec = 20;


   /** KeSWRV_t_DrvCAN_TmeOut: Swerve Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Milliseconds). 
     */
    public static final int KeSWRV_t_DrvCAN_TmeOut = 0;




  /********************************************/
  /*  Rotation Zero Offset Learn Algorithm    */
 	/********************************************/	 	

   /** KeSWRV_Deg_RZL_AngSwpCourse: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Relative Sweep Angle for Coarse Zero Search. 
     */
    public static final double KeSWRV_Deg_RZL_AngSwpCourse = 90;

   /** KeSWRV_Deg_RZL_AngSwpFine: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Relative Sweep Angle for Fine Zero Search. 
     */
    public static final double KeSWRV_Deg_RZL_AngSwpFine = 10;

   /** KeSWRV_r_RZL_PwrSwpCourse: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Power Level for Coarse Zero Search. 
     */    
    public static final double KeSWRV_r_RZL_PwrSwpCourse = 0.20;

   /** KeSWRV_r_RZL_PwrSwpFine: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Power Level for Fine Zero Search. 
     */    
    public static final double KeSWRV_r_RZL_PwrSwpFine = 0.05;

   /** KeSWRV_t_RZL_ZeroDtctThrsh: Swerve Drive System - Rotation Control
     * Encoder Zero Learn Zero Detection Time. 
     */    
    public static final double KeSWRV_t_RZL_ZeroDtctThrsh = 0.250;



}
