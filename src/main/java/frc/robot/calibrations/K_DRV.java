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
public class K_DRV {



  /**************************************************/
  /*  Tank Drive Configuration Calibrations       */
 	/**************************************************/	 	

	  /** KeDRV_b_DebugEnbl: Swerve Drive System Enable
     *  Calibartion to send data to dashbord to debug.
     */
    public static final boolean KeDRV_b_DebugEnbl = false;


  /**************************************************/
  /*  Swerve Drive General System Calibrations      */
 	/**************************************************/	 	

	  /** KeDRV_r_CntlrDeadBandFwd: Tank Drive System: Normalized Power
     * Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized for Forward/Rearward Commands.
     * If the Absolute value of the Input is below the Threshold it
     * will be ignored (i.e. set to Zero).
     */
    public static final double KeDRV_r_CntlrDeadBandFwd = 0.2;

	  /** KeDRV_r_DB_CntlrThrshRot: Tank Drive System: Normalized Power
     * Dead-Band Threshold that must be met before a X-Box Controller
     * Joystick Input is recognized for Forward/Rearward Commands.
     * If the Absolute value of the Input is below the Threshold it
     * will be ignored (i.e. set to Zero).
     */
    public static final double KeDRV_r_DB_CntlrThrshRot = 0.2;


	  /** KeDRV_r_DrvRqstOvrrdFwd: Tank Drive System: Scaled Normalized Power
     * Threshold that at and above Forward/Rearward Drive Control will
     * Override any Rotation Request for stability.  Does not include
     * Closed-Loop Drive Heading Control.
     */
    public static final double KeDRV_r_DrvRqstOvrrdFwd = 0.4;


	  /** KeDRV_r_DrvRqstOvrrdRot: Tank Drive System: Scaled Normalized Power
     * Threshold that at and above Rotation Drive Control will Override
     * Override Forward/Rearward Requests for stability, as long as the
     * Forward/Rearward Request is less than KeDRV_r_DrvRqstOvrrdFwd.
     */
    public static final double KeDRV_r_DrvRqstOvrrdRot = 0.4;



  /*******************************************************/
  /*  Tank Drive-System Design Mechanical Parameters     */
 	/*******************************************************/	 	
	
	  /** KeDRV_Cf_DrvEncdrCntsPerInchHi: Tank Drive System - Conversion
     * Factor of the Number of Drive Motor Encoder Counts of Rotation
     * per 1 Linear inch of Drive Wheel travel in Low Gear Ratio.
     */
    public static final double KeDRV_Cf_DrvEncdrCntsPerInchHi = 35.6;


	  /** KeDRV_Cf_DrvEncdrCntsPerInchLo: Tank Drive System - Conversion
     * Factor of the Number of Drive Motor Encoder Counts of Rotation
     * per 1 Linear inch of Drive Wheel travel in Low Gear Ratio.
     */
    public static final double KeDRV_Cf_DrvEncdrCntsPerInchLo = 35.6;


	  /** KeDRV_Cnt_DrvEncdrCntsPerRev: Tank Drive System: Number
     * of Encoder Counts Per One Revolution of the Drive Motors
     */
    public static final double KeDRV_Cnt_DrvEncdrCntsPerRev = 360;


	  /** KeDRV_l_DrvWhlDistPerRot: Drive System: Number of lineal
     * distance travaled per one rotation of Drive Wheel Rotation
     */
    public static final double KeDRV_l_DrvWhlDistPerRot = 4.0 * Math.PI; // 12.57142857


    /** KeDRV_r_DrvMtrEncdrToWhlRatLo: Tank Drive System: Number of
     * Drive Control Motor Encoder Rotations to Wheel Rotations
     * Ratio for Low Gear Ration.
     */
    public static final double KeDRV_r_DrvMtrEncdrToWhlRatLo = 11.87016;


	  /** KeDRV_r_DrvMtrEncdrToWhlRatHi: Tank Drive System: Number of
     * Drive Control Motor Encoder Rotations to Wheel Rotations
     * Ratio for High Gear Ratio.
     */
    public static final double KeDRV_r_DrvMtrEncdrToWhlRatHi = 8.166667;



  /**************************************************/
  /*  Tank Drive Drive Control PID Coefficients   */
 	/**************************************************/	 	

	  /** KeDRV_K_DrvProp: Tank Drive System Drive Control
     * Proporational Control Gain. 
     */
    public static final double KeDRV_K_DrvProp = 2.0;


	  /** KeDRV_K_DrvPropIntgl: Tank Drive System Drive Control
     * Integral Control Gain. 
     */
    public static final double KeDRV_K_DrvIntgl = 0.0;


	  /** KeDRV_K_DrvDeriv: Tank Drive System Drv Control
     * Derivative Control Gain. 
     */
    public static final double KeDRV_K_DrvDeriv = 0.0;


	  /** KeDRV_K_DrvFdFwd: Tank Drive System Drive Control
     * Feed Fowrward Control Gain. 
     */
    public static final double KeDRV_K_DrvFdFwd = 0.0;


	  /** KeDRV_n_DrvIntglErrMaxEnbl: Tank Drive System Drive Control
     * Maximum Error Signal Threshold (absolute value) that Integral
     * correction will applied.  Error Signal must be within band 
     * (+/- postive value). (rpm/100ms) 
     */
    public static final int KeDRV_n_DrvIntglErrMaxEnbl = 0;


	  /** KeDRV_n_Drv_MM_CruiseVel: Tank Drive System: This is the
     * peak target velocity that the motion magic curve generator can
     * use for the Drive Control Speed Closed-Loop. (RPM/100msec) 
     */
    public static final int KeDRV_n_Drv_MM_CruiseVel = 9000;


	  /** KeDRV_a_Drv_MM_MaxAccel: Tank Drive System: Sets the Motion
     * Magic Acceleration. This is the target acceleration that the motion
     * magic curve generator can use for the Drive Control Speed
     * Closed-Loop.  (RPM/100msec/sec)
     */
    public static final int KeDRV_a_Drv_MM_MaxAccel = 13000;


	  /** KeDRV_r_DrvNormOutMax: Tank Drive System Drive Control
     * Normalized Output Power Maximum Limit. 
     */
    public static final int KeDRV_r_DrvNormOutMax = 1;


	  /** KeDRV_r_DrvNormOutMin: Tank Drive System Drive Control
     * Normalized Output Power Minimum Limit. 
     */
    public static final int KeDRV_r_DrvNormOutMin = -1;
 


  /******************************************/
  /*  Swerve Drive Current Driver Limits    */
 	/******************************************/	 	

   /** KeDRV_I_DrvDrvrLmtMaxPri: Tank Drive System: Drive Control
     * Motor Driver Maximum Current Limit for Primary Driver,
     * Supply Current Limit.
     */
    public static final double KeDRV_I_DrvDrvrLmtMaxPri = 60;

 
   /** KeDRV_I_DrvDrvrLmtMaxSec: Tank Drive System Drive Control
     * Motor Driver Maximum Current Limit for Secondary Driver,
     * Stator Current Limit. 
     */
    public static final double KeDRV_I_DrvDrvrLmtMaxSec = 60;


   /** KeDRV_t_DrvCAN_TmeOut: Tank Drive System Rotation Control
     * Motor Driver CAN Message Time Out Threshold (in Milliseconds). 
     */
    public static final double KeDRV_t_DrvCAN_TmeOut = 0.5;



  /******************************************************/
  /*  Swerve Drive Control: Closed Loop Drive Control   */
 	/******************************************************/	 	

  /** KeDRV_Cnt_CL_DrvErrTgtDB_Fwd: Tank Drive System - 
    * Closed Loop Control Drive Control Error Target Deadband
    * Below which Closed-Loop will be considered Complete for
    * Longitudinal control (Forward/Rearward).
    */    
    public static final int KeDRV_Cnt_CL_DrvErrTgtDB_Fwd = 2;


  /** KeDRV_Deg_CL_DrvErrTgtDB_Rot: Tank Drive System - 
    * Closed Loop Control Drive Control Error Target Deadband
    * Below which Closed-Loop will be considered Complete for
    * Rotational control (CW/CCW).
    */    
    public static final double KeDRV_Deg_CL_DrvErrTgtDB_Rot = 1;

    
  /** KeDRV_t_CL_DrvSyncThrshFwd: Tank Drive System - 
    * Closed Loop Control Drive Control Sync Time for
    * Longitudinal Drive control (Forward/Reward).
    */    
    public static final double KeDRV_t_CL_DrvSyncThrshFwd = 0.100;


  /** KeDRV_t_CL_DrvSyncThrshFwd: Tank Drive System - 
    * Closed Loop Control Drive Control Sync Time for
    * Longitudinal Drive control (Forward/Reward).
    */    
    public static final double KeDRV_t_CL_DrvSyncThrshRot = 0.100;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Proportional Gain for
    * Longitudinal Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_PropGx_Long = 1;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Integral Gain for
    * Longitudinal Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_IntglGx_Long = 0.001;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Derivative Gain for
    * Longitudinal Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_DerivGx_Long = 0;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Proportional Gain for
    * Rotational Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_PropGx_Rot = 1;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Integral Gain for
    * Rotational Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_IntglGx_Rot = 0.001;


  /** KeDRV_k_CL_PropGx_Long: Tank Drive System - 
    * Closed Loop Control Derivative Gain for
    * Rotational Drive control (Forward/Reward).
    */
    public static final double KeDRV_k_CL_DerivGx_Rot = 0;

}
