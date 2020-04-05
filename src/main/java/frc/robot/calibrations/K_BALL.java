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
public class K_BALL {


  /**************************************************/
  /*  Ball System Configuration Calibrations       */
 	/**************************************************/	 	

	  /** KeBAL_b_DebugEnbl: Ball Feed System Enable
     *  Calibration to send data to dashboard to debug.
     */
    public static final boolean KeBAL_b_DebugEnbl = false;



  /**************************************************/
  /*  Ball System General System Calibrations       */
 	/**************************************************/	 	

	  /** KeBAL_r_NormPwrAdvLoad: Ball Feed System: Normalized Power
      * Commanded to the Ball Advance Motor during Ball Loading Commands.
      */
    public static final double KeBAL_r_NormPwrAdvLoad = 0.375;


    /** KeBAL_r_NormPwrAdvShoot: Ball Feed System: Normalized Power
      * Commanded to the Ball Advance Motor during Ball Loading Commands.
      */
    public static final double KeBAL_r_NormPwrAdvShoot = 1.0;




	  /** KeBAL_r_NormPwrIntakeLoad: Ball Feed System: Normalized Power
      * Commanded to the Ball Intake Motor during Ball Loading Commands.
      */
    public static final double KeBAL_r_NormPwrIntakeLoad = 1.0;


	  /** KeBAL_r_NormPwrIntakeShoot: Ball Feed System: Normalized Power
      * Commanded to the Ball Intake Motor during Ball Loading Commands.
      */
    public static final double KeBAL_r_NormPwrIntakeShoot = 1.0;




	  /** KeBAL_r_NormPwrIntakeClear: Ball Feed System: Normalized Power
      * Commanded to the Ball Intake Motor during Ball Clearing Commands.
      */
    public static final double KeBAL_r_NormPwrIntakeClear = 0.75;  // Presently not used

}
