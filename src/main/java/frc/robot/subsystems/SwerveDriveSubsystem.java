package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.controller.PIDController;


public class SwerveDriveSubsystem extends HolonomicDrivetrainSubsystem {

	public enum TeRotDirctn {
        CW,
        CCW;
   }


	private static final double VeSDRV_r_ChassisTrkRat = Math.sqrt(Math.pow(K_SWRV.KeSWRV_l_ChassisWhlBase, 2) + Math.pow(K_SWRV.KeSWRV_l_ChassisTrkWdth, 2));

	private SwerveDriveModule[] SwrvDrvMod = new SwerveDriveModule[] {                            
		new SwerveDriveModule(SwrvMap.RtFt, new CANSparkMax(Constants.SWRV_FR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_RT_DRV), 0), //real:298 practice: 56
		new SwerveDriveModule(SwrvMap.LtFt, new CANSparkMax(Constants.SWRV_FR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_LT_DRV), 0), //real:355 practice: 190
		new SwerveDriveModule(SwrvMap.LtRr, new CANSparkMax(Constants.SWRV_RR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_LT_DRV), 0), //real:293 practice: 59
		new SwerveDriveModule(SwrvMap.RtRr, new CANSparkMax(Constants.SWRV_RR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_RT_DRV), 0)  //real:390 practice: 212
	};

	private DigitalInput[] VaSDRV_b_RotZeroDtcd = new DigitalInput[] {
		new DigitalInput(Constants.SWRV_ZERO_FR_RT),
		new DigitalInput(Constants.SWRV_ZERO_FR_LT),
		new DigitalInput(Constants.SWRV_ZERO_RR_LT),
		new DigitalInput(Constants.SWRV_ZERO_RR_RT)
	};

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);


	boolean VeSDRV_b_SwrvRotRqstActv;
	
	double[] VaSDRV_Deg_RotAngActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngAct        = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngCalcRaw    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtd = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCorr   = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcLtch   = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngTgt        = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrTgt        = new double[SwrvMap.NumOfCaddies];      

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
			VaSDRV_Deg_RotAngActRaw[i] = 0;
			VaSDRV_Deg_RotAngAct[i] = 0;
			VaSDRV_Deg_RotAngCalcLtch[i] = 0;
			VaSDRV_v_DrvSpdCalcRaw[i] = 0;
			VaSDRV_v_DrvSpdCalcCnvrtd[i] = 0;
			VaSDRV_Deg_RotAngTgt[i] = 0;
			VaSDRV_r_RotEncdrTgt[i] = 0;
		}
    
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
	public void HolonomicDrv(double Le_r_PwrLong, double Le_r_PwrLat, double Le_r_PwrRat) {
        boolean Le_b_RotTqtUpdCond = false;

		if (Math.abs(Le_r_PwrRat) >= K_SWRV.KeSWRV_r_CntlrDeadBandThrsh) {
			VeSDRV_b_SwrvRotRqstActv = true;		
			VeSDRV_r_PwrLong = 0;
			VeSDRV_r_PwrLat  = 0;
			VeSDRV_r_PwrRot  = Le_r_PwrRat;
		}
		else {
			VeSDRV_b_SwrvRotRqstActv = false;
		    VeSDRV_r_PwrLong = Le_r_PwrLong;
		    VeSDRV_r_PwrLat  = Le_r_PwrLat;
			VeSDRV_r_PwrRot  = 0;
		}

		VeSDRV_r_PwrLong *= getSpdMult();
		VeSDRV_r_PwrLat  *= getSpdMult();
		/*
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad); 
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}
		*/

		if (Math.abs(Le_r_PwrLong) > 0.05 || Math.abs(Le_r_PwrLat) > 0.05 || Math.abs(Le_r_PwrRat) > 0.05) {
            Le_b_RotTqtUpdCond = true;
		}

		double a = Le_r_PwrLat  - Le_r_PwrRat * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double b = Le_r_PwrLat  + Le_r_PwrRat * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double c = Le_r_PwrLong - Le_r_PwrRat * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);
		double d = Le_r_PwrLong + Le_r_PwrRat * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);

		VaSDRV_Deg_RotAngCalcRaw =  new double[]{
				Math.atan2(b, c) * 180 / Math.PI,
				Math.atan2(b, d) * 180 / Math.PI,
				Math.atan2(a, d) * 180 / Math.PI,
				Math.atan2(a, c) * 180 / Math.PI
		};

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		    if (VaSDRV_Deg_RotAngCalcRaw[i] < 0.0 ) {
			    /* Negative = Counter Clockwise (-0 to -180] Convert to Positive (360-180] */
			    VaSDRV_Deg_RotAngCalcCnvrtd[i] = 360 + VaSDRV_Deg_RotAngCalcRaw[i];
		    }
		    else {
			    VaSDRV_Deg_RotAngCalcCnvrtd[i] = VaSDRV_Deg_RotAngCalcRaw[i];
		    }
			VaSDRV_Deg_RotAngCalcCorr[i] = SwrvDrvMod[i].adjustRotTgtAng(VaSDRV_Deg_RotAngCalcCnvrtd[i]);

			if (Le_b_RotTqtUpdCond == true) {
				VaSDRV_Deg_RotAngCalcLtch[i] = VaSDRV_Deg_RotAngCalcCorr[i];
			} 
	    }


		VaSDRV_v_DrvSpdCalcRaw = new double[]{
				Math.sqrt(b * b + c * c),
				Math.sqrt(b * b + d * d),
				Math.sqrt(a * a + d * d),
				Math.sqrt(a * a + c * c)
		};

		VeSDRV_v_DrvSpdMax = 0.0;

		// Limit VaSDRV_v_DrvSpdCalcRaw[x]
		for (double LeSDRV_v_SpdTmp : VaSDRV_v_DrvSpdCalcRaw) { 
			if (LeSDRV_v_SpdTmp > VeSDRV_v_DrvSpdMax) {
				VeSDRV_v_DrvSpdMax = LeSDRV_v_SpdTmp;
			}
		}

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			if (VeSDRV_v_DrvSpdMax > 1.0) {
			  VaSDRV_v_DrvSpdCalcCnvrtd[i] = VaSDRV_v_DrvSpdCalcRaw[i]/VeSDRV_v_DrvSpdMax;
			  }
		    else {
			  VaSDRV_v_DrvSpdCalcCnvrtd[i] = VaSDRV_v_DrvSpdCalcRaw[i];
			} 
			VaSDRV_Deg_RotAngActRaw[i] = SwrvDrvMod[i].getRotActAngRaw();
			VaSDRV_Deg_RotAngAct[i] = SwrvDrvMod[i].cnvrtRotActAng(VaSDRV_Deg_RotAngActRaw[i]);

		SmartDashboard.putString("Drv Mtr Dir Prev " + i , SwrvDrvMod[i].getDrvMtrDirctn().toString());
		}

       dtrmnDrvMtrDirctn(SwrvMap.LtFt, SwrvMap.LtRr, VeSDRV_b_SwrvRotRqstActv);
       dtrmnDrvMtrDirctn(SwrvMap.RtFt, SwrvMap.RtRr, VeSDRV_b_SwrvRotRqstActv);


		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			VaSDRV_Deg_RotAngTgt[i] = SwrvDrvMod[i].adjustRotTgtAng(VaSDRV_Deg_RotAngCalcLtch[i]);
			VaSDRV_r_RotEncdrTgt[i] = SwrvDrvMod[i].calcRotEncdrTgt(VaSDRV_Deg_RotAngTgt[i]);
			SwrvDrvMod[i].setRotEncdrTgt(VaSDRV_r_RotEncdrTgt[i]);
		}
		setDrvMtrSpd(SwrvMap.RtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.RtFt]);
		setDrvMtrSpd(SwrvMap.LtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.LtFt]);

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
		double Le_Deg_RotAngTgtFt;
		double Le_Deg_RotAngActFt;
        double Le_Deg_RotErrAbsFt;      
        boolean Le_b_DirctnFlipCondFt;
        double Le_Deg_RotErrFtTemp;      

        double Le_Deg_RotAngTgtRr;
        double Le_Deg_RotAngActRr;
        double Le_Deg_RotErrAbsRr;      
        boolean Le_b_DirctnFlipCondRr;
        double Le_Deg_RotErrRrTemp;      

        boolean Le_b_DirctnFlipCondComb;
        boolean Le_b_DirctnFlipTrig;


        // Get Rotation Target Angle (0 - 360)
        Le_Deg_RotAngTgtFt = VaSDRV_Deg_RotAngCalcLtch[Le_i_ModIdxFt]; 
        Le_Deg_RotAngTgtRr = VaSDRV_Deg_RotAngCalcLtch[Le_i_ModIdxRr]; 
        
        // Get Rotation Current Angle (0-360)
        Le_Deg_RotAngActFt = VaSDRV_Deg_RotAngAct[Le_i_ModIdxFt];
        Le_Deg_RotAngActRr = VaSDRV_Deg_RotAngAct[Le_i_ModIdxRr];

	    // Calc Rotation Angle Error Absolute Value (0-360)
        Le_Deg_RotErrAbsFt = Math.abs(Le_Deg_RotAngTgtFt - Le_Deg_RotAngActFt);
        Le_Deg_RotErrAbsRr = Math.abs(Le_Deg_RotAngTgtRr - Le_Deg_RotAngActRr);

	    // Determine Flip COndition of Each Caddy Independently
		Le_b_DirctnFlipCondFt = evalDirctnFlipCond(Le_Deg_RotErrAbsFt);
		Le_b_DirctnFlipCondRr = evalDirctnFlipCond(Le_Deg_RotErrAbsRr);

    	SmartDashboard.putNumber("Dir Tgt Ft (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotAngTgtFt);
		SmartDashboard.putNumber("Dir Act Ft (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotAngActFt);
    	SmartDashboard.putNumber("Dir Tgt Rr (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotAngTgtRr);
		SmartDashboard.putNumber("Dir Act Rr (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotAngActRr);
    	SmartDashboard.putNumber("Dir Ang Err Fr (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotErrAbsFt);
		SmartDashboard.putNumber("Dir Ang Err Rr (Deg) " + Le_i_ModIdxFt,  Le_Deg_RotErrAbsRr);
    	SmartDashboard.putBoolean("Dir Flip Cond Fr " + Le_i_ModIdxFt,  Le_b_DirctnFlipCondFt);
		SmartDashboard.putBoolean("Dir Flip Cond Rr " + Le_i_ModIdxFt,  Le_b_DirctnFlipCondRr);

		if ((Le_b_DirctnFlipCondFt == false) && (Le_b_DirctnFlipCondRr == false)) {
            Le_b_DirctnFlipCondComb = false;
		}
		else if ((Le_b_DirctnFlipCondFt == true) && (Le_b_DirctnFlipCondRr == true)) {
            Le_b_DirctnFlipCondComb = true;
		}
	    else {
			 /* If Error is over 180 it is faster to rotate the other direction
				So do the math to deterimine shortest distance based on rotating
				either direction. */

			 /* Front Drive: Convert [180-360) Error to [-180 to -0} error */
			 if (Le_Deg_RotErrAbsFt >= 180) {
				Le_Deg_RotErrFtTemp = Le_Deg_RotErrAbsFt - 360;
			 }
			 else {
				Le_Deg_RotErrFtTemp = Le_Deg_RotErrAbsFt;
			 }
			 /* Front Drive: Get Absolute Value of Delta to Target */
			 Le_Deg_RotErrFtTemp = Math.abs(Le_Deg_RotErrFtTemp);


			 /* Rear Drive: Convert [180-360) Error to [-180 to -0} error */
			 if (Le_Deg_RotErrAbsRr >= 180) {
				Le_Deg_RotErrRrTemp = Le_Deg_RotErrAbsRr - 360;
			 }
			 else {
				Le_Deg_RotErrRrTemp = Le_Deg_RotErrAbsRr;
			 }
			 /* Rear Drive: Get Absolute Value of Delta to Target */
			 Le_Deg_RotErrRrTemp = Math.abs(Le_Deg_RotErrRrTemp);
			 
			 /* Flip Conditions Met: Front but not Back */
			 if (Le_b_DirctnFlipCondFt == true) {
			 /* If the conditions determined to flip Front Drive
				but not Back Drive: Only flip directions if the 
				delta the front drive had prior to inverting
				direction is greatuer than the delta that the rear
				drive will have to rotate after invertind direction. */
				Le_Deg_RotErrRrTemp = Math.abs(Le_Deg_RotErrRrTemp - 180);
				if (Le_Deg_RotErrFtTemp > Le_Deg_RotErrRrTemp)
					Le_b_DirctnFlipCondComb = true;
				else
				    Le_b_DirctnFlipCondComb = false; 
			 }
			 /* Flip Conditions Met: Back but not Front */
		     else /* (Le_b_DirctnFlipCondRr == true)	*/ {
				/* If the conditions determined to flip Back Drive
				   but not flip Front Drive: Only flip directions if
				   the delta the back drive had prior to inverting
				   direction is greatuer than the delta that the front
				   drive will have to rotate after invertind direction. */
				Le_Deg_RotErrFtTemp = Math.abs(Le_Deg_RotErrFtTemp - 180);
				if (Le_Deg_RotErrRrTemp > Le_Deg_RotErrFtTemp)
					Le_b_DirctnFlipCondComb = true;
				else
				    Le_b_DirctnFlipCondComb = false; 				
			 }	
		}
		
    	SmartDashboard.putBoolean("Dir Flip Cond Comb " + Le_i_ModIdxFt,  Le_b_DirctnFlipCondComb);

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


    public void resetCaddyRotZeroOfst(int Le_i_SwrvModIdx) {
        SwrvDrvMod[Le_i_SwrvModIdx].resetRotAngZeroOfst();
    }


    public void resetSwrvDrvRotZeroOfst() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)
            SwrvDrvMod[i].resetRotAngZeroOfst();
    }






    /*************************************************/
	/*     Subsystem Data Interfaces                 */
	/*************************************************/

	public boolean getRotZeroDtctd(int Le_i_SwrvModIdx) {
	     return(VaSDRV_b_RotZeroDtcd[Le_i_SwrvModIdx].get());
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

	for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
	  
	   SmartDashboard.putNumber("Tgt Ang Raw " + i ,       VaSDRV_Deg_RotAngCalcRaw[i]);
	   SmartDashboard.putNumber("Tgt Ang Corr " + i ,      VaSDRV_Deg_RotAngCalcLtch[i]);
	   SmartDashboard.putNumber("Tgt Encdr Revs " + i ,    VaSDRV_r_RotEncdrTgt[i]);
       SmartDashboard.putNumber("Act Encdr Revs " + i ,    SwrvDrvMod[i].getRotEncdrActPstn());	   
       SmartDashboard.putNumber("Act Ang Raw " + i ,       VaSDRV_Deg_RotAngActRaw[i]);
       SmartDashboard.putNumber("Act Ang Corr " + i ,      VaSDRV_Deg_RotAngAct[i]);
	   SmartDashboard.putNumber("Tgt Spd Raw " + i ,       VaSDRV_v_DrvSpdCalcRaw[i]);
	   SmartDashboard.putNumber("Tgt Spd Cnvrtd " + i ,    VaSDRV_v_DrvSpdCalcCnvrtd[i]);
       SmartDashboard.putNumber("Drv Encdr Cnts " + i ,    SwrvDrvMod[i].getDrvEncdrDelt());	   
	   SmartDashboard.putString("Drv Mtr Dir " + i ,       SwrvDrvMod[i].getDrvMtrDirctn().toString());
	   SmartDashboard.putBoolean("Drv Mtr Dir Trig " + i , SwrvDrvMod[i].getDrvMtrDirctnTrig());
       SmartDashboard.putNumber("Rot Zero Ofst " + i ,     SwrvDrvMod[i].getRotAngZeroOfst());
       SmartDashboard.putNumber("Rot Mtr Cur " + i ,       SwrvDrvMod[i].getRotMtrCurr());
	   
	   }

	}


}

