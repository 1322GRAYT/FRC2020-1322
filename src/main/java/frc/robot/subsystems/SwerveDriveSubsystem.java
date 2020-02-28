package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;
import frc.robot.subsystems.SwerveDriveModule.TeRotDirctn;
import frc.robot.subsystems.SwerveDriveModule.Te_RZL_St;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;


public class SwerveDriveSubsystem extends HolonomicDrivetrainSubsystem {

	private static final double VeSDRV_r_ChassisTrkRat = Math.sqrt(Math.pow(K_SWRV.KeSWRV_l_ChassisWhlBase, 2) + Math.pow(K_SWRV.KeSWRV_l_ChassisTrkWdth, 2));

	private SwerveDriveModule[] SwrvDrvMod = new SwerveDriveModule[] {                            
		new SwerveDriveModule(SwrvMap.RtFt, new CANSparkMax(Constants.SWRV_FR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_RT_DRV), new DigitalInput(Constants.SWRV_ZERO_FR_RT), 0), //real:298 practice: 56
		new SwerveDriveModule(SwrvMap.LtFt, new CANSparkMax(Constants.SWRV_FR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_LT_DRV), new DigitalInput(Constants.SWRV_ZERO_FR_LT), 0), //real:355 practice: 190
		new SwerveDriveModule(SwrvMap.LtRr, new CANSparkMax(Constants.SWRV_RR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_LT_DRV), new DigitalInput(Constants.SWRV_ZERO_RR_LT), 0), //real:293 practice: 59
		new SwerveDriveModule(SwrvMap.RtRr, new CANSparkMax(Constants.SWRV_RR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_RT_DRV), new DigitalInput(Constants.SWRV_ZERO_RR_RT), 0)  //real:390 practice: 212
	};

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);


	private boolean VeSDRV_b_SwrvRotRqstActv;
	private boolean VeSDRV_b_RZL_Rqst;
	private boolean VeSDRV_b_RZL_RqstPrev;
	private boolean VeSDRV_b_RZL_Cmplt;
	

	double[] VaSDRV_r_RotEncdrActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrActCorr    = new double[SwrvMap.NumOfCaddies];	
	double[] VaSDRV_Deg_RotAngActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngActNorm    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngActCnvrtd  = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngCalcRaw    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtd = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcLtch   = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngTgt        = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrTgt        = new double[SwrvMap.NumOfCaddies];      
    double[] VaSDRV_r_RotEncdrTgtCorr    = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_v_DrvSpdCalcRaw      = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_v_DrvSpdCalcCnvrtd   = new double[SwrvMap.NumOfCaddies];

	
	private double VeSDRV_v_DrvSpdMax;

	private double VeSDRV_r_MtrSpdMult;

	private double VeSDRV_r_PwrLong;
	private double VeSDRV_r_PwrLat;
	private double VeSDRV_r_PwrRot;



	/*******************************/
	/* Subsystem Constructor       */
	/*******************************/
	public SwerveDriveSubsystem() {
		zeroGyro(); 

		SwrvDrvMod[SwrvMap.LtRr].getDrvMtr().follow(SwrvDrvMod[SwrvMap.LtFt].getDrvMtr());
		SwrvDrvMod[SwrvMap.RtRr].getDrvMtr().follow(SwrvDrvMod[SwrvMap.RtFt].getDrvMtr());

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
		  VaSDRV_r_RotEncdrActRaw[i]    = 0;
		  VaSDRV_r_RotEncdrActCorr[i]   = 0;
		  VaSDRV_Deg_RotAngActRaw[i]    = 0;
		  VaSDRV_Deg_RotAngActNorm[i]   = 0;
		  VaSDRV_Deg_RotAngActCnvrtd[i] = 0;
		  VaSDRV_Deg_RotAngCalcLtch[i]  = 0;
		  VaSDRV_v_DrvSpdCalcRaw[i]     = 0;
		  VaSDRV_v_DrvSpdCalcCnvrtd[i]  = 0;
		  VaSDRV_Deg_RotAngTgt[i]       = 0;
		  VaSDRV_r_RotEncdrTgt[i]       = 0;
		  VaSDRV_r_RotEncdrTgtCorr[i]   = 0;
		}

	    VeSDRV_b_SwrvRotRqstActv = false;
		VeSDRV_b_RZL_Rqst        = false;
		VeSDRV_b_RZL_RqstPrev    = false;
	    VeSDRV_b_RZL_Cmplt       = false;

		VeSDRV_v_DrvSpdMax  = 0.0;
		VeSDRV_r_MtrSpdMult = 1.0;

		VeSDRV_r_PwrLong = 0.0;
		VeSDRV_r_PwrLat  = 0.0;
		VeSDRV_r_PwrRot  = 0.0;
	}



    /*********************************************/
	/*     Subsystem Device Interfaces           */
	/*********************************************/

	public AHRS getNavX() {
		return mNavX;
	}

	public SwerveDriveModule getSwerveModule(int i) {
		return SwrvDrvMod[i];
	}




    /*********************************************/
	/*     NAV/Gyro Interfaces                   */
	/*********************************************/

	public double getGyroAngle() {
		return (mNavX.getAngle() - getAdjustmentAngle());
	}

	public double getGyroRate() {
		return mNavX.getRate();
	}

	public double getRawGyroAngle() {
		return mNavX.getAngle();
	}

	public double getYaw()
	{
		return mNavX.getAngle();
	}




    /*********************************************/
	/*     Subsystem Method Definitions          */
	/*********************************************/

	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	}



	@Override
	public void HolonomicDrv(double Le_r_PwrLong, double Le_r_PwrLat, double Le_r_PwrRot) {
		boolean Le_b_RotAngUpdPwrCond;

        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
		  SmartDashboard.putNumber("X-Box Pwr Long " , Le_r_PwrLong);
		  SmartDashboard.putNumber("X-Box Pwr Lat "  , Le_r_PwrLat);
		  SmartDashboard.putNumber("X-Box Pwr Rot "  , Le_r_PwrRot);
		}
		
		VeSDRV_r_PwrLong = applyDB_NormPwr(Le_r_PwrLong, K_SWRV.KeSWRV_r_CntlrDeadBandThrsh);
		VeSDRV_r_PwrLat  = applyDB_NormPwr(Le_r_PwrLat,  K_SWRV.KeSWRV_r_CntlrDeadBandThrsh);
		VeSDRV_r_PwrRot  = applyDB_NormPwr(Le_r_PwrRot,  K_SWRV.KeSWRV_r_CntlrDeadBandThrsh);

		if (Math.abs(Le_r_PwrRot) > K_SWRV.KeSWRV_r_CntlrDeadBandThrsh) {
			VeSDRV_b_SwrvRotRqstActv = true;		
			VeSDRV_r_PwrLong = 0;
			VeSDRV_r_PwrLat  = 0;
		}
		else {
			VeSDRV_b_SwrvRotRqstActv = false;
			VeSDRV_r_PwrRot  = 0;
		}

		VeSDRV_r_PwrLong *= getSpdMult();
		VeSDRV_r_PwrLat  *= getSpdMult();
		/*
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = VeSDRV_r_PwrLong * Math.cos(angleRad) + VeSDRV_r_PwrLat * Math.sin(angleRad); 
			VeSDRV_r_PwrLat = -VeSDRV_r_PwrLong * Math.sin(angleRad) + VeSDRV_r_PwrLat * Math.cos(angleRad);
			VeSDRV_r_PwrLong = temp;
		}
		*/


        /* Process Swerve Drive Inverse Kinematics */

		double a = VeSDRV_r_PwrLat  - VeSDRV_r_PwrRot * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double b = VeSDRV_r_PwrLat  + VeSDRV_r_PwrRot * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double c = VeSDRV_r_PwrLong - VeSDRV_r_PwrRot * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);
		double d = VeSDRV_r_PwrLong + VeSDRV_r_PwrRot * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);

		VaSDRV_Deg_RotAngCalcRaw =  new double[]{
		        Math.atan2(b, c) * 180 / Math.PI,
			    Math.atan2(b, d) * 180 / Math.PI,
			    Math.atan2(a, d) * 180 / Math.PI,
			    Math.atan2(a, c) * 180 / Math.PI
		};

		VaSDRV_v_DrvSpdCalcRaw = new double[]{
			    Math.sqrt(b * b + c * c),
			    Math.sqrt(b * b + d * d),
			    Math.sqrt(a * a + d * d),
			    Math.sqrt(a * a + c * c)
		};


        /* Process Rotation Calculated Target Angle and Actual Angle */

		if (Math.abs(VeSDRV_r_PwrLong) > K_SWRV.KeSWRV_r_CntlrDeadBandThrsh ||
			Math.abs(VeSDRV_r_PwrLat)  > K_SWRV.KeSWRV_r_CntlrDeadBandThrsh ||
			Math.abs(VeSDRV_r_PwrRot)  > K_SWRV.KeSWRV_r_CntlrDeadBandThrsh ) {
		  Le_b_RotAngUpdPwrCond = true;
		}
		else {
		  Le_b_RotAngUpdPwrCond = false;
		}
     
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		  /* Convert [-180 to 0 to 180] Angle where 0 was Forward to [0 to 180 to 360] Where 180 is Forward */
		  VaSDRV_Deg_RotAngCalcCnvrtd[i] = VaSDRV_Deg_RotAngCalcRaw[i] + 180;			
		  /* Update Latched Rotation Angle value if Input Power is High enough */
		  if (Le_b_RotAngUpdPwrCond) {
			VaSDRV_Deg_RotAngCalcLtch[i] = VaSDRV_Deg_RotAngCalcCnvrtd[i];
		  } 
		  /* Get Rotation AEncoder Position Raw */
		  VaSDRV_r_RotEncdrActRaw[i] = SwrvDrvMod[i].getRotEncdrActPstn();
		  /* Apply Rotation Encoder Position Zero Offset Correction to Raw Position */
          VaSDRV_r_RotEncdrActCorr[i] = SwrvDrvMod[i].correctRotEncdrActPstn(VaSDRV_r_RotEncdrActRaw[i]); 
		  /* Calculate Caddy Angle from Encoder Position (can include multiple revolutions) */         
		  VaSDRV_Deg_RotAngActRaw[i] = SwrvDrvMod[i].getRotActAngRaw();
		  /* Normalize the Actual Rotation Angle to a Single Rotation */
		  VaSDRV_Deg_RotAngActNorm[i] = SwrvDrvMod[i].normRotActAng(VaSDRV_Deg_RotAngActRaw[i]);
		  /* Convert [-180 to 0 to 180] Angle where 0 was Forward to [0 to 180 to 360] Where 180 is Forward */
		  VaSDRV_Deg_RotAngActCnvrtd[i] = SwrvDrvMod[i].cnvrtRotActAng(VaSDRV_Deg_RotAngActNorm[i]);
	    }


        /* Process Drive Target Speed */

		/* Peak Detection for values within VaSDRV_v_DrvSpdCalcRaw[x] for Drive Speed Normalization below */
		VeSDRV_v_DrvSpdMax = 0.0;
		for (double LeSDRV_v_SpdTmp : VaSDRV_v_DrvSpdCalcRaw) { 
		  if (LeSDRV_v_SpdTmp > VeSDRV_v_DrvSpdMax) {
			VeSDRV_v_DrvSpdMax = LeSDRV_v_SpdTmp;
		  }
		}

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		  if (VeSDRV_v_DrvSpdMax > 1.0) {
			/* Normalize to 1 based on the Max Speed if the Max Speed exceeds 1.0 */
		    VaSDRV_v_DrvSpdCalcCnvrtd[i] = VaSDRV_v_DrvSpdCalcRaw[i]/VeSDRV_v_DrvSpdMax;
		  }
		  else {
		    VaSDRV_v_DrvSpdCalcCnvrtd[i] = VaSDRV_v_DrvSpdCalcRaw[i];
		  } 

		SmartDashboard.putString("Drv Mtr Dir Prev " + i , SwrvDrvMod[i].getDrvMtrDirctn().toString());
		}


       /* Determine if Drive Motor Direction Switch should be done. */
       dtrmnDrvMtrDirctn(SwrvMap.LtFt, SwrvMap.LtRr, VeSDRV_b_SwrvRotRqstActv);
       dtrmnDrvMtrDirctn(SwrvMap.RtFt, SwrvMap.RtRr, VeSDRV_b_SwrvRotRqstActv);


       /* Calculate Rotation Angle Target and Command Target Rotation Positions */
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			VaSDRV_Deg_RotAngTgt[i] = SwrvDrvMod[i].calcRotTgtAng(VaSDRV_Deg_RotAngCalcLtch[i]);
			VaSDRV_r_RotEncdrTgt[i] = SwrvDrvMod[i].calcRotEncdrTgt(VaSDRV_Deg_RotAngTgt[i]);
			VaSDRV_r_RotEncdrTgtCorr[i] = SwrvDrvMod[i].correctRotEncdrTgtPstn(VaSDRV_r_RotEncdrTgt[i]);
			SwrvDrvMod[i].setRotEncdrTgt(VaSDRV_r_RotEncdrTgt[i]);
		}

		/* Command Target Drive Speeds */
		setDrvMtrSpd(SwrvMap.RtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.RtFt]);
		setDrvMtrSpd(SwrvMap.LtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.LtFt]);

		/* Update SmartDashboard with Data */
        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
		    updateSmartDash();	
        }

	}



	/** Method: dtrmnDrvMtrDirctn - Swerve Drive System: Determination of
	  * the Drive Motor Direction for the Drive Motor that runs a Coupled
	  * Front/Rear Pair of Swerve Caddies, to obtain the Desired Swerve
	  * Drive Caddy Rotation Angles for the pair in the most efficient manner.
	  * This is for a Crab-Drive Swerve Drive Set-Up. 
	  * @param Le_i_ModIdxFt        (int: swerve module index for Front Caddy)
	  * @param Le_i_ModIdxRr        (int: swerve module index for Rear Caddy)
	  * @param Le_b_SwrvRotRqstActv (boolean: swerve caddy rotation request active)
      */   
	  private void dtrmnDrvMtrDirctn(int Le_i_ModIdxFt, int Le_i_ModIdxRr, boolean Le_b_SwrvRotRqstActv) {
		double  Le_Deg_RotAngTgtFt;
		double  Le_Deg_RotAngActFt;
        double  Le_Deg_RotErrAbsFt;      
        boolean Le_b_DirctnFlipCondFt;
        double  Le_Deg_RotErrMinAbsFt;      

        double  Le_Deg_RotAngTgtRr;
        double  Le_Deg_RotAngActRr;
        double  Le_Deg_RotErrAbsRr;      
        boolean Le_b_DirctnFlipCondRr;
        double  Le_Deg_RotErrMinAbsRr;      

        boolean Le_b_DirctnFlipCondComb;
        boolean Le_b_DirctnFlipTrig;

		// Get Rotation Target Angle [0 - 360]
		Le_Deg_RotAngTgtFt = (VaSDRV_Deg_RotAngCalcLtch[Le_i_ModIdxFt]); 
		Le_Deg_RotAngTgtRr = (VaSDRV_Deg_RotAngCalcLtch[Le_i_ModIdxRr]);
		// Correct for Reverse Drive Motor Direction if Necessary 
		if (SwrvDrvMod[Le_i_ModIdxFt].getDrvMtrDirctn() == TeMtrDirctn.Rwd) {
		  Le_Deg_RotAngTgtFt = (Le_Deg_RotAngTgtFt + 180) % 360;
		}
		if (SwrvDrvMod[Le_i_ModIdxRr].getDrvMtrDirctn() == TeMtrDirctn.Rwd) {
          Le_Deg_RotAngTgtRr = (Le_Deg_RotAngTgtRr + 180) % 360; 
		}

        // Get Rotation Current Angle [ 0-360 ]
        Le_Deg_RotAngActFt = VaSDRV_Deg_RotAngActCnvrtd[Le_i_ModIdxFt];
        Le_Deg_RotAngActRr = VaSDRV_Deg_RotAngActCnvrtd[Le_i_ModIdxRr];

	    // Calc Rotation Angle Error Absolute Value (0-360)
        Le_Deg_RotErrAbsFt = Math.abs(Le_Deg_RotAngTgtFt - Le_Deg_RotAngActFt);
        Le_Deg_RotErrAbsRr = Math.abs(Le_Deg_RotAngTgtRr - Le_Deg_RotAngActRr);

	    // Determine Flip Condition of Each Caddy Independently
		Le_b_DirctnFlipCondFt = evalDirctnFlipCond(Le_Deg_RotErrAbsFt);
		Le_b_DirctnFlipCondRr = evalDirctnFlipCond(Le_Deg_RotErrAbsRr);

		if ((Le_b_DirctnFlipCondFt == false) && (Le_b_DirctnFlipCondRr == false)) {
            Le_b_DirctnFlipCondComb = false;
		}
		else if ((Le_b_DirctnFlipCondFt == true) && (Le_b_DirctnFlipCondRr == true)) {
            Le_b_DirctnFlipCondComb = true;
		}
	    else {
            /* Front Wheel Determine Shortest Distance to Target Rotating either Direction */
			Le_Deg_RotErrMinAbsFt = calcDirctnDistMin(Le_Deg_RotErrAbsFt);

            /* Front Wheel Determine Shortest Distance to Target Rotating either Direction */
			Le_Deg_RotErrMinAbsRr = calcDirctnDistMin(Le_Deg_RotErrAbsRr);
			 
			/* Flip Conditions Met: Front but not Back */
			if (Le_b_DirctnFlipCondFt == true) {
			  Le_b_DirctnFlipCondComb = evalDirctnFlipCondCmplx(Le_Deg_RotErrMinAbsFt, Le_Deg_RotErrMinAbsRr);  
			}
			/* Flip Conditions Met: Back but not Front */
		    else /* (Le_b_DirctnFlipCondRr == true)	*/ {
			  Le_b_DirctnFlipCondComb = evalDirctnFlipCondCmplx(Le_Deg_RotErrMinAbsRr, Le_Deg_RotErrMinAbsFt);  
			}	
		}
		
		if ((K_SWRV.KeSWRV_b_DrvMtrRotDirctnInvertEnbl == false) ||
			(SwrvDrvMod[Le_i_ModIdxFt].getDrvMtrDirctnUpdInhb() == true) ||
		    (SwrvDrvMod[Le_i_ModIdxRr].getDrvMtrDirctnUpdInhb() == true) ||
		    (K_SWRV.KeSWRV_b_DrvMtrRotDirctnInvertInhbRot == true) && (Le_b_SwrvRotRqstActv == true)) {
            Le_b_DirctnFlipTrig = false;
        }
        else {
        Le_b_DirctnFlipTrig = Le_b_DirctnFlipCondComb;
        }


		if (Le_b_DirctnFlipTrig)
		  {
		  SwrvDrvMod[Le_i_ModIdxFt].invertDrvMtrDirctn();
		  SwrvDrvMod[Le_i_ModIdxRr].invertDrvMtrDirctn();
		  }

		SwrvDrvMod[Le_i_ModIdxFt].setDrvMtrDirctnTrig(Le_b_DirctnFlipTrig);
		SwrvDrvMod[Le_i_ModIdxRr].setDrvMtrDirctnTrig(Le_b_DirctnFlipTrig);
	
    }


	/** Method: evalDirctnFlipCond - Swerve Drive System: Evaluate if it
	  * would be more efficient to for a Caddy to achieve the Target 
	  * Rotation Angle by Reversing the Drive Direction, which would
	  * effectively rotate the Actual Angle by 180 degrees.
	  * @param Le_Deg_RotAngErr      (double: swerve module caddy rotation angle error)
	  * @return Le_b_SwrvRotRqstActv (boolean: swerve caddy rotation request active)
      */   
	  private boolean evalDirctnFlipCond(double Le_Deg_RotAngErr) {
		boolean Le_b_DirctnFlipCondMet = false;

		if ((Le_Deg_RotAngErr > 90) && (Le_Deg_RotAngErr < 270))
            Le_b_DirctnFlipCondMet = true;

		return (Le_b_DirctnFlipCondMet);
	  }


	/** Method: calcDirctnMinDist - Swerve Drive System: If Error is 
	 * over 180, it is faster to rotate around the other direction, 
	 * so do the math to deterimine shortest distance based on 
	 * rotating either direction and use the smallest error.
	 * @param Le_Deg_RotErrAbs      (double:  swerve module caddy rotation angle error)
	 * @return Le_Deg_RotErrAbsMin  (boolean: swerve caddy rotation angle error min)
     */   
	  private double calcDirctnDistMin(double Le_Deg_RotErrAbs) {
		double Le_Deg_RotErrMin, Le_Deg_RotErrMinAbs;

			 /* Convert [180-360) Error to [-180 to -0) Error */
			 if (Le_Deg_RotErrAbs >= 180) {
				Le_Deg_RotErrMin = Le_Deg_RotErrAbs - 360;
			 }
			 else {
				Le_Deg_RotErrMin = Le_Deg_RotErrAbs;
			 }
			 /* Get Absolute Value of Delta to Target */
			 Le_Deg_RotErrMinAbs = Math.abs(Le_Deg_RotErrMin);

		return (Le_Deg_RotErrMinAbs);
	  }


	/** Method: evalDirctnFlipCondCmplx - Swerve Drive System: If the conditions
	 * determined to flip one Drive but not flip the other Drive: Only flip
	 * directions, if the delta the active wheel (wheel that meets the flip
	 * conditions) has prior to inverting direction is greatuer than the
	 * delta that the passive wheel will have to rotate after inverting
	 * directions, otherwise don't flip.
	 * @param  Le_Deg_ActvWhlErrAbs  (double:  swerve module Flip Active Wheel Angle Error Absolute )
	 * @param  Le_Deg_PsvWhlErrAbs   (double:  swerve module Flip Passive Wheel Angle Error Absolute)
	 * @return Le_b_CmplxDrvWhlFlipCondMet  (boolean: Drive Wheel Flip Condition Met Indicator)
     */   
	private boolean evalDirctnFlipCondCmplx(double Le_Deg_ActvWhlErrAbs, double Le_Deg_PsvWhlErrAbs) {
	  double  Le_Deg_PsvWhlErrAbsFlipped;
	  boolean Le_b_CmplxDrvWhlFlipCondMet;

	  Le_Deg_PsvWhlErrAbsFlipped = (Le_Deg_PsvWhlErrAbs + 180);
	  if (Le_Deg_PsvWhlErrAbsFlipped > 180) {
	    Le_Deg_PsvWhlErrAbsFlipped = 180 - Le_Deg_PsvWhlErrAbsFlipped;
	  }
	  if (Le_Deg_ActvWhlErrAbs > Le_Deg_PsvWhlErrAbsFlipped)
	    Le_b_CmplxDrvWhlFlipCondMet = true;
	  else
	    Le_b_CmplxDrvWhlFlipCondMet = false;
	  
	  return (Le_b_CmplxDrvWhlFlipCondMet);
	}


    /*****************************************************/
	/*     Subsystem Command Methods and Interfaces     */
	/*****************************************************/

	public double applyDB_NormPwr(double Le_r_PwrRqstRaw, double Le_r_DB_Thrsh) {
		double Le_r_PwrRqstLim, Le_r_PwrRqstDBAppl;
		double Le_r_DB_ThrshProt;
		double Le_r_RescaleNum, Le_r_RescaleDenom;
	   
		Le_r_PwrRqstLim = Math.min(Le_r_PwrRqstRaw, 1.0);
		Le_r_PwrRqstLim = Math.max(Le_r_PwrRqstLim, -1.0);
	   
		Le_r_DB_ThrshProt = Math.abs(Le_r_DB_Thrsh);
		Le_r_DB_ThrshProt = Math.min(Le_r_DB_ThrshProt, 1.0);

		if (Math.abs(Le_r_PwrRqstLim) < Le_r_DB_ThrshProt) {
			Le_r_PwrRqstDBAppl = 0;  
		  }
		else /* (Math.abs(Le_r_PwrRqstLim) >= Le_r_DB_ThrshProt) */ {
			Le_r_RescaleDenom = 1 - Le_r_DB_ThrshProt;
			if (Le_r_PwrRqstLim >= 0.0) {
			  Le_r_RescaleNum = Le_r_PwrRqstLim - Le_r_DB_ThrshProt;
			}
			else /* (Le_r_PwrRqstLim < 0.0) */ {
			  Le_r_RescaleNum = Le_r_PwrRqstLim + Le_r_DB_ThrshProt;
			}
			Le_r_PwrRqstDBAppl = Le_r_RescaleNum/Le_r_RescaleDenom;
		}
		
		return (Le_r_PwrRqstDBAppl);
	}    


	public double applyDB(double Le_r_PwrRqstRaw, double Le_r_DB_Thrsh) {
		double Le_r_PwrRqstDBAppl;
		double Le_r_DB_ThrshProt;
	   	   
		Le_r_DB_ThrshProt = Math.abs(Le_r_DB_Thrsh);

		if (Math.abs(Le_r_PwrRqstRaw) < Le_r_DB_ThrshProt) {
			Le_r_PwrRqstDBAppl = 0;  
		}
		else /* (Math.abs(Le_r_PwrRqstRaw) >= Le_r_DB_ThrshProt) */ {
			if (Le_r_PwrRqstRaw >= 0.0)
				Le_r_PwrRqstDBAppl = Le_r_PwrRqstRaw - Le_r_DB_ThrshProt;
			else /* (Le_r_PwrRqstRaw < 0.0) */
				Le_r_PwrRqstDBAppl = Le_r_PwrRqstRaw + Le_r_DB_ThrshProt;
		}
		
		return (Le_r_PwrRqstDBAppl);
	}    



	public void haltSwerveDrive(){
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
			SwrvDrvMod[i].setRotMtrPwr(0);
            SwrvDrvMod[i].setDrvMtrPwr(0);
		}
	}


	public void runSwerveCaddyAtPwr(int Le_i_SwrvBnkIdx, TeMtrDirctn Le_e_MtrDirctn, double Le_r_PwrLvl) {

		double Le_r_MtrDirctnSclr = (Le_e_MtrDirctn == TeMtrDirctn.Rwd ? -1 : 1); 

		if (Le_i_SwrvBnkIdx == SwrvMap.RtSd) {
			SwrvDrvMod[SwrvMap.RtFt].setDrvMtrPwr(Le_r_MtrDirctnSclr * Le_r_PwrLvl);
			SwrvDrvMod[SwrvMap.RtRr].setDrvMtrPwr(Le_r_MtrDirctnSclr * Le_r_PwrLvl);
			SwrvDrvMod[SwrvMap.LtFt].setDrvMtrPwr(0);
			SwrvDrvMod[SwrvMap.LtRr].setDrvMtrPwr(0);
		}
		else {
			SwrvDrvMod[SwrvMap.LtFt].setDrvMtrPwr(Le_r_MtrDirctnSclr * Le_r_PwrLvl);
			SwrvDrvMod[SwrvMap.LtRr].setDrvMtrPwr(Le_r_MtrDirctnSclr * Le_r_PwrLvl);
			SwrvDrvMod[SwrvMap.RtFt].setDrvMtrPwr(0);
			SwrvDrvMod[SwrvMap.RtRr].setDrvMtrPwr(0);
		}

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
	        SwrvDrvMod[i].setRotMtrPwr(0);
		}
	}
	

	public double getSwerveCaddyAng(int Le_i_SwrvMod) {
	    return(SwrvDrvMod[Le_i_SwrvMod].getRotActAngRaw());
	}


	public boolean getSDRV_RZL_Rqst() {
		return (VeSDRV_b_RZL_Rqst);
		}
	  

	public void setSDRV_RZL_Rqst(boolean Le_b_RZL_Rqst) {
	    VeSDRV_b_RZL_Rqst = Le_b_RZL_Rqst;
	}


    public boolean getSDRV_RZL_Cmplt() {
	  return (VeSDRV_b_RZL_Cmplt);
	}

	public void setSDRV_RZL_Cmplt(boolean Le_b_RZL_Cmplt) {
	  VeSDRV_b_RZL_Cmplt = Le_b_RZL_Cmplt;
	}


    public void mngSDRV_RZL_SchedTask() {
	  boolean Le_b_RZL_InitTrig = false;
	  Te_RZL_St Le_e_St_RtFt, Le_e_St_LtFt, Le_e_St_LtRr, Le_e_St_RtRr;
	 
		if (VeSDRV_b_RZL_Rqst == true) {
		  if (VeSDRV_b_RZL_RqstPrev = false) {
			Le_b_RZL_InitTrig = true;
		  }

		setDrvMtrSpdsZero();

		Le_e_St_RtFt = SwrvDrvMod[SwrvMap.RtFt].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		Le_e_St_LtFt = SwrvDrvMod[SwrvMap.LtFt].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		Le_e_St_LtRr = SwrvDrvMod[SwrvMap.LtRr].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		Le_e_St_RtRr = SwrvDrvMod[SwrvMap.RtRr].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);			

		if ((Le_e_St_RtFt == Te_RZL_St.Cmplt) && (Le_e_St_LtFt == Te_RZL_St.Cmplt) &&
		    (Le_e_St_LtRr == Te_RZL_St.Cmplt) && (Le_e_St_RtRr == Te_RZL_St.Cmplt)) {
		  VeSDRV_b_RZL_Cmplt = true;
		}
	  }
	  else {
        SwrvDrvMod[SwrvMap.RtFt].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
        SwrvDrvMod[SwrvMap.LtFt].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
        SwrvDrvMod[SwrvMap.LtRr].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
        SwrvDrvMod[SwrvMap.RtRr].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
	  }

	  VeSDRV_b_RZL_RqstPrev = VeSDRV_b_RZL_Rqst;
	}
   

	public void sweepSwerveCaddyToAng(int Le_i_SwrvModIdx, TeRotDirctn Le_e_RotDirctn, double Le_r_PwrLvl) {
		double Le_r_RotDirctnSclr = 1;

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
			if (i == Le_i_SwrvModIdx)  {
			  if (Le_e_RotDirctn == TeRotDirctn.CCW)
			      Le_r_RotDirctnSclr = -1;
			  SwrvDrvMod[i].setRotMtrPwr(Le_r_PwrLvl * Le_r_RotDirctnSclr);
			  SwrvDrvMod[i].setDrvMtrPwr(0);
			}
			else
			  SwrvDrvMod[i].setRotMtrPwr(0);
			  SwrvDrvMod[i].setDrvMtrPwr(0);
		}
	}





	public void driveForwardAtSpd(double Le_Deg_HdgAng, double Le_r_PwrLvl){
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(Le_r_PwrLvl, 0, 0);
	}


	public void driveForwardToDist(double Le_l_TgtPos, double Le_Deg_HdgAng, double Le_r_PwrLvl){
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(Le_r_PwrLvl, 0, Le_Deg_HdgAngErr);
	}



	public void driveSidewaysAtSpd(double Le_Deg_HdgAng, double Le_r_PwrLvl) {
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(0, Le_r_PwrLvl, 0);
	}


	public void driveSidewaysToDist(double Le_l_TgtPos, double Le_Deg_HdgAng, double Le_r_PwrLvl) {
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(0, Le_r_PwrLvl, Le_Deg_HdgAngErr);
	}



	public void driveRotateAtSpd(TeRotDirctn Le_e_RotDirctn, double Le_r_PwrLvl) {
        double Le_r_RotPwr;
		if (Le_e_RotDirctn == TeRotDirctn.CCW)
		  Le_r_RotPwr = Le_r_PwrLvl * -1;
		else
		  Le_r_RotPwr = Le_r_PwrLvl;
		Le_r_RotPwr = Math.min(Le_r_RotPwr, 1);
		Le_r_RotPwr = Math.max(Le_r_RotPwr, -1);
		HolonomicDrv(0, 0, Le_r_RotPwr);
	}


	public void driveRotateToAng(TeRotDirctn Le_e_RotDirctn, double Le_Deg_RotTgt, double Le_r_PwrLvl) {
		double Le_Deg_RotErr = ((Le_Deg_RotTgt - mNavX.getYaw()) / 180)*10;
		Le_Deg_RotErr = Math.min(Le_Deg_RotErr, 1);

		Le_Deg_RotErr = Math.max(Le_Deg_RotErr, -1);
		if (Math.abs(Le_Deg_RotErr) < 5 )
		  Le_r_PwrLvl *= 0.25;
		HolonomicDrv(0, 0, Le_r_PwrLvl);
	}


	public void cntrlBrkInSwrvDrv(double Le_r_PwrRot, double Le_r_PwrDrv) {	
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
       	   SwrvDrvMod[i].setRotMtrPwr(Le_r_PwrRot);
		}
		SwrvDrvMod[SwrvMap.RtFt].setDrvMtrPwr(Le_r_PwrDrv);
		SwrvDrvMod[SwrvMap.LtFt].setDrvMtrPwr(Le_r_PwrDrv);
	}




	public double calcDrvPosErr(double Le_l_DrvDistTgtInches) {
		return Le_l_DrvDistTgtInches - SwrvDrvMod[SwrvMap.RtFt].getDrvDist();
	}


	public void cmndDrvsToEncdrPos(double Le_r_EncdrPos) {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].setRotEncdrTgt(Le_r_EncdrPos);
		}
	}


	@Override
	public void stopDrvMtrs() {
		for (int i = 0; i < SwrvMap.NumOfBanks; i++ ) {
			setDrvMtrSpd(i, 0);
		}
	}


	public void resetDrvEncdrs() {
		for (int i = 0; i < SwrvMap.NumOfBanks; i++ ) {
			SwrvDrvMod[i].resetDrvEncdrPstn();
		}
	}


	public void resetRotEncdrs() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].resetRotEncdr();
		}
	}


	public void resetCaddyRotEncdr(int Le_i_SwrvModIdx) {
		SwrvDrvMod[Le_i_SwrvModIdx].resetRotEncdr();
	}




    /*************************************************/
	/*     Subsystem Data Interfaces                 */
	/*************************************************/

	
	public boolean getRotZeroDtctd(int Le_i_SwrvModIdx) {
	     return(SwrvDrvMod[Le_i_SwrvModIdx].getRotEncdrZeroDtctd());
	}


	public void setDrvMtrSpdsZero() {
	  for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		SwrvDrvMod[i].setDrvMtrPwr(0);
		SwrvDrvMod[i].setDrvMtrPwr(0);
		SwrvDrvMod[i].setDrvMtrPwr(0);
		SwrvDrvMod[i].setDrvMtrPwr(0);
	  }
    }





    public void setDrvMtrSpd(int Le_i_MtrBnk, double Le_r_DrvPwr) {
		TeMtrDirctn Le_e_DrvMtrDirctn;
		double Le_r_DirctnSclr;
		
		if (Le_i_MtrBnk == SwrvMap.RtSd)
		    Le_e_DrvMtrDirctn = SwrvDrvMod[SwrvMap.RtFt].getDrvMtrDirctn();
	    else
 		    Le_e_DrvMtrDirctn = SwrvDrvMod[SwrvMap.LtFt].getDrvMtrDirctn();

        Le_r_DirctnSclr = (Le_e_DrvMtrDirctn == TeMtrDirctn.Rwd ? -1 : 1); 

		if (Le_i_MtrBnk == SwrvMap.RtSd) {
			SwrvDrvMod[SwrvMap.RtFt].setDrvMtrPwr(Le_r_DirctnSclr * Le_r_DrvPwr);
			SwrvDrvMod[SwrvMap.RtRr].setDrvMtrPwr(Le_r_DirctnSclr * Le_r_DrvPwr);
		}
		else {
			SwrvDrvMod[SwrvMap.LtFt].setDrvMtrPwr(Le_r_DirctnSclr * Le_r_DrvPwr);
			SwrvDrvMod[SwrvMap.LtRr].setDrvMtrPwr(Le_r_DirctnSclr * Le_r_DrvPwr);
		}
    }


	public double getSpdMult() {
		return VeSDRV_r_MtrSpdMult;
	}

	public void setSpdMult(double LeSDRV_r_MtrSpdMult) {
		this.VeSDRV_r_MtrSpdMult = LeSDRV_r_MtrSpdMult;
	}	




    /*************************************************/
	/*     Subsystem Instrumenation Display          */
	/*************************************************/

	private void updateSmartDash() {
	/* Print to SmartDashboard */

	SmartDashboard.putNumber("Cntrlr Pwr Long " , VeSDRV_r_PwrLong);
	SmartDashboard.putNumber("Cntrlr Pwr Lat " ,  VeSDRV_r_PwrLat);
	SmartDashboard.putNumber("Cntrlr Pwr Rot " ,  VeSDRV_r_PwrRot);

	SmartDashboard.putBoolean("RZL Rqst " , VeSDRV_b_RZL_Rqst);
	SmartDashboard.putBoolean("RZL Cmplt " , VeSDRV_b_RZL_Cmplt);

	for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {

	   SmartDashboard.putString("RZL St " + i ,            SwrvDrvMod[i].getRotEncdrZeroLearnSt().toString());
      

	   SmartDashboard.putNumber("Calc Ang Raw " + i ,      VaSDRV_Deg_RotAngCalcRaw[i]);
	   SmartDashboard.putNumber("Calc Ang Cnvrtd " + i ,   VaSDRV_Deg_RotAngCalcCnvrtd[i]);
	   SmartDashboard.putNumber("Calc Ang Ltch " + i ,     VaSDRV_Deg_RotAngCalcLtch[i]);

       SmartDashboard.putNumber("Tgt Ang Raw " + i ,       VaSDRV_Deg_RotAngTgt[i]);	   
	   SmartDashboard.putNumber("Tgt Encdr Raw " + i ,     VaSDRV_r_RotEncdrTgt[i]);
	   SmartDashboard.putNumber("Tgt Encdr Corr " + i ,    VaSDRV_r_RotEncdrTgtCorr[i]);

	   SmartDashboard.putNumber("Act Encdr Raw " + i ,     VaSDRV_r_RotEncdrActRaw[i]);
	   SmartDashboard.putNumber("Act Encdr Corr " + i ,    VaSDRV_r_RotEncdrActCorr[i]); 
	   SmartDashboard.putNumber("Act Ang Raw " + i ,       VaSDRV_Deg_RotAngActRaw[i]);
	   SmartDashboard.putNumber("Act Ang Norm " + i ,      VaSDRV_Deg_RotAngActNorm[i]);
	   SmartDashboard.putNumber("Act Ang Cnvrtd " + i ,    VaSDRV_Deg_RotAngActCnvrtd[i]);

       SmartDashboard.putNumber("Rot Zero Ofst " + i ,     SwrvDrvMod[i].getRotEncdrZeroOfst());
       SmartDashboard.putNumber("Rot Mtr Cur " + i ,       SwrvDrvMod[i].getRotMtrCurr());
	
	   SmartDashboard.putNumber("Tgt Spd Raw " + i ,       VaSDRV_v_DrvSpdCalcRaw[i]);
	   SmartDashboard.putNumber("Tgt Spd Cnvrtd " + i ,    VaSDRV_v_DrvSpdCalcCnvrtd[i]);
       SmartDashboard.putNumber("Drv Encdr Cnts " + i ,    SwrvDrvMod[i].getDrvEncdrDelt());	   
	   SmartDashboard.putString("Drv Mtr Dir " + i ,       SwrvDrvMod[i].getDrvMtrDirctn().toString());
	   SmartDashboard.putBoolean("Drv Mtr Dir Trig " + i , SwrvDrvMod[i].getDrvMtrDirctnTrig());
	   
	   }

	}

}

