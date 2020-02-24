package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;

public class SwerveDriveSubsystem extends HolonomicDrivetrainSubsystem {

	public enum TeRotDirctn {
        CW,
        CCW;
   }


	private static final double VeSDRV_r_ChassisTrkRat = Math.sqrt(Math.pow(K_SWRV.KeSWRV_l_ChassisWhlBase, 2) + Math.pow(K_SWRV.KeSWRV_l_ChassisTrkWdth, 2));

	private SwerveDriveModule[] SwrvDrvMod = new SwerveDriveModule[] {                            
		new SwerveDriveModule(SwrvMap.RtFt, new CANSparkMax(Constants.SWRV_FR_RT_ROT, MotorType.kBrushless),  0), //real:298 practice: 56
		new SwerveDriveModule(SwrvMap.LtFt, new CANSparkMax(Constants.SWRV_FR_LT_ROT, MotorType.kBrushless),  0), //real:355 practice: 190
		new SwerveDriveModule(SwrvMap.LtRr, new CANSparkMax(Constants.SWRV_RR_LT_ROT, MotorType.kBrushless),  0), //real:293 practice: 59
		new SwerveDriveModule(SwrvMap.RtRr, new CANSparkMax(Constants.SWRV_RR_RT_ROT, MotorType.kBrushless),  0)  //real:390 practice: 212
	};

	private TalonFX[] Va_h_DrvMtr = new TalonFX[] {
		new TalonFX(Constants.SWRV_RR_LT_DRV),
		new TalonFX(Constants.SWRV_RR_LT_DRV)	
	};

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);


	boolean VeSDRV_b_SwrvRotRqstActv;
	
	double[] VaSDRV_Deg_RotAngActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngAct        = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngCalcRaw    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtd = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCorr   = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtdLtch = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcLtch   = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngTgt        = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrTgt        = new double[SwrvMap.NumOfCaddies];
      
	double[] VaSDRV_r_DrvEncdrZeroPstn   = new double[SwrvMap.NumOfBanks];    

	double[] VaSDRV_v_DrvSpdCalcRaw      = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_v_DrvSpdCalcCnvrtd   = new double[SwrvMap.NumOfCaddies];

	
	private double VeSDRV_v_DrvSpdMax;

	private double VeSDRV_r_MtrSpdMult;

    /* Subsystem Constructor */
	public SwerveDriveSubsystem() {
		zeroGyro(); 


		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
			VaSDRV_Deg_RotAngActRaw[i] = 0;
			VaSDRV_Deg_RotAngAct[i] = 0;
			VaSDRV_Deg_RotAngCalcCnvrtdLtch[i] = 0;
			VaSDRV_Deg_RotAngCalcLtch[i] = 0;
			VaSDRV_Deg_RotAngTgt[i] = 0;
			VaSDRV_r_RotEncdrTgt[i] = 0;
		}

		for (int i = 0; i < SwrvMap.NumOfBanks; i++)  {
		    Va_h_DrvMtr[i].setInverted(true);
		    Va_h_DrvMtr[i].setNeutralMode(NeutralMode.Brake);
		    resetDrvZeroPstn(i);
		    VaSDRV_r_DrvEncdrZeroPstn[i] = (double)Va_h_DrvMtr[i].getSelectedSensorPosition();
		}
    
		VeSDRV_v_DrvSpdMax = 0.0;
		VeSDRV_r_MtrSpdMult = 1.0;

		
        /*****************************************************************/
        /* Drive Control PID Controller Configurations                   */
		/*****************************************************************/
		for (int i = 0; i < SwrvMap.NumOfBanks; i++) {
            Va_h_DrvMtr[i].configFactoryDefault();
            Va_h_DrvMtr[i].setInverted(false);
            Va_h_DrvMtr[i].setSensorPhase(false);

		    // set PID coefficients
			Va_h_DrvMtr[i].config_kP(0, K_SWRV.KeSWRV_K_DrvProp);
		    Va_h_DrvMtr[i].config_kI(0, K_SWRV.KeSWRV_K_DrvIntgl);
		    Va_h_DrvMtr[i].config_kD(0, K_SWRV.KeSWRV_K_DrvDeriv);
		    Va_h_DrvMtr[i].config_IntegralZone(0, K_SWRV.KeSWRV_e_DrvIntglErrMaxEnbl);
		    Va_h_DrvMtr[i].config_kF(0, K_SWRV.KeSWRV_K_DrvFdFwd);
            Va_h_DrvMtr[i].configMotionCruiseVelocity(K_SWRV.KeSWRV_n_Drv_MM_CruiseVel);
            Va_h_DrvMtr[i].configMotionAcceleration(K_SWRV.KeSWRV_a_Drv_MM_MaxAccel);
//          Va_h_DrvMtr[i].configIntegratedSensorAbsoluteRange([K_SWRV.KeSWRV_r_DrvNormOutMin, K_SWRV.KeSWRV_r_DrvNormOutMax), (int)0);

            // Set Idle Mode
            Va_h_DrvMtr[i].setNeutralMode(NeutralMode.Brake);

            // Set Sensor Type
            Va_h_DrvMtr[i].configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
 
            // Set amperage limits
            Va_h_DrvMtr[i].configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxPri, 15, 0.5));
            Va_h_DrvMtr[i].configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxSec, 25 ,1.0));
//          Va_h_DrvMtr[i].setCANTimeout(K_SWRV.KeSWRV_t_DrvCAN_TmeOut);
		}
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

	public TalonFX getDrvMtr(int i) {
			return Va_h_DrvMtr[i];
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
	public void HolonomicDrv(double Le_r_LongPwr, double Le_r_LatPwr, double Le_r_RotPwr) {
        boolean Le_b_RotTqtUpdCond = false;
		
        VeSDRV_b_SwrvRotRqstActv = Math.abs(Le_r_RotPwr) > .1;

		Le_r_LongPwr *= getSpdMult();
		Le_r_LatPwr  *= getSpdMult();
		/*
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad); 
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}
		*/

		if (Math.abs(Le_r_LongPwr) > 0.05 || Math.abs(Le_r_LatPwr) > 0.05 || Math.abs(Le_r_RotPwr) > 0.05) {
            Le_b_RotTqtUpdCond = true;
		}

		double a = Le_r_LatPwr - Le_r_RotPwr * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double b = Le_r_LatPwr + Le_r_RotPwr * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double c = Le_r_LongPwr - Le_r_RotPwr * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);
		double d = Le_r_LongPwr + Le_r_RotPwr * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);

		VaSDRV_Deg_RotAngCalcRaw =  new double[]{
				Math.atan2(b, c) * 180 / Math.PI,
				Math.atan2(b, d) * 180 / Math.PI,
				Math.atan2(a, d) * 180 / Math.PI,
				Math.atan2(a, c) * 180 / Math.PI
		};

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		    if (VaSDRV_Deg_RotAngCalcRaw[i] < 0.0 ) {
			    /* Negative = Counter Clockwise - 0 to -180 Convert to Positive 0 to 36n0*/
			    VaSDRV_Deg_RotAngCalcCnvrtd[i] = 360 - VaSDRV_Deg_RotAngCalcRaw[i];
		    }
		    else {
			    VaSDRV_Deg_RotAngCalcCnvrtd[i] = VaSDRV_Deg_RotAngCalcRaw[i];
		    }
			VaSDRV_Deg_RotAngCalcCorr[i] = SwrvDrvMod[i].adjustRotTgtAng(VaSDRV_Deg_RotAngCalcCnvrtd[i]);

			if (Le_b_RotTqtUpdCond == true) {
				VaSDRV_Deg_RotAngCalcCnvrtdLtch[i] = VaSDRV_Deg_RotAngCalcCnvrtd[i];
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
			VaSDRV_Deg_RotAngActRaw[i] = SwrvDrvMod[i].getRotActAngRaw();
			VaSDRV_Deg_RotAngAct[i] = SwrvDrvMod[i].cnvrtRotActAng(VaSDRV_Deg_RotAngActRaw[i]);
		}

       dtrmnDrvMtrDirctn(SwrvMap.LtFt, SwrvMap.LtRr, VeSDRV_b_SwrvRotRqstActv);
       dtrmnDrvMtrDirctn(SwrvMap.RtFt, SwrvMap.RtRr, VeSDRV_b_SwrvRotRqstActv);


		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
            VaSDRV_Deg_RotAngTgt[i] = SwrvDrvMod[i].adjustRotTgtAng(VaSDRV_Deg_RotAngCalcCnvrtdLtch[i]);
			SwrvDrvMod[i].setRotEncdrTgt(VaSDRV_Deg_RotAngTgt[i]);
		}
		setDrvMtrSpd(SwrvMap.RtSd, VaSDRV_v_DrvSpdCalcRaw[SwrvMap.RtFt]);
		setDrvMtrSpd(SwrvMap.LtSd, VaSDRV_v_DrvSpdCalcRaw[SwrvMap.LtFt]);

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
		

		if ((SwrvDrvMod[Le_i_ModIdxFt].getDrvMtrDirctnUpdInhb() == true) ||
		    (SwrvDrvMod[Le_i_ModIdxRr].getDrvMtrDirctnUpdInhb() == true) ||
		    (K_SWRV.KeSWRV_b_DrvMtrRotDirctnInvertInhb == true) && (Le_b_SwrvRotRqstActv == true)) {
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





    /*************************************************/
	/*     Subsystem Command and Interfaces     */
	/*************************************************/


	public void haltSwerveDrive(){

		HolonomicDrv(0, 0, 0);
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
	/*	
		this.drvFrontLeft.set(ControlMode.PercentOutput, fwdPower);
		this.drvFrontRight.set(ControlMode.PercentOutput, fwdPower);
	
		rotRearLeft.set(rotPower);
		rotFrontLeft.set(rotPower);
		rotRearRight.set(rotPower);
		rotFrontRight.set(rotPower);
	*/
	}


	public double calcDrvPosErr(double d1) {
		return d1 - getDrvDist(SwrvMap.RtSd);
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
			Va_h_DrvMtr[i].setSelectedSensorPosition(0);
		}
	}

	public void resetRotEncdrs() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].resetRotEncdr();
		}
	}


    /*************************************************/
	/*     Subsystem Data Interfaces                 */
	/*************************************************/


    public void setDrvMtrSpd(int Le_i_MtrBnk, double Le_r_MtrSpd) {
		TeMtrDirctn Le_e_DrvMtrDirctn;
		
		if (Le_i_MtrBnk == SwrvMap.RtSd)
		    Le_e_DrvMtrDirctn = SwrvDrvMod[SwrvMap.RtFt].getDrvMtrDirctn();
	    else
 		    Le_e_DrvMtrDirctn = SwrvDrvMod[SwrvMap.LtFt].getDrvMtrDirctn();

        double Le_r_DirctnSclr = (Le_e_DrvMtrDirctn == TeMtrDirctn.Rwd ? -1 : 1); 

        Va_h_DrvMtr[Le_i_MtrBnk].set(ControlMode.Velocity, (Le_r_DirctnSclr * Le_r_MtrSpd));
    }


    public void resetDrvZeroPstn(int Le_i_MtrBnk) {
        VaSDRV_r_DrvEncdrZeroPstn[Le_i_MtrBnk] = (double)Va_h_DrvMtr[Le_i_MtrBnk].getSelectedSensorPosition();
    }


    public double getDrvEncdrDelt(int Le_i_MtrBnk) {
        return ((double)Va_h_DrvMtr[Le_i_MtrBnk].getSelectedSensorPosition() - VaSDRV_r_DrvEncdrZeroPstn[Le_i_MtrBnk]);
    }


    public double getDrvInchesPerEncdrCnts(double Le_r_DrvEncdrNormCnts) {
         return Le_r_DrvEncdrNormCnts / K_SWRV.KeSWRV_Cf_DrvMtrEncdrCntsToInch;
    }


    public int getDrvEncdrCntsPerInches(double Le_l_DrvWhlDistInches) {
         return (int) Math.round(Le_l_DrvWhlDistInches * K_SWRV.KeSWRV_Cf_DrvMtrEncdrCntsToInch);
    }


    public double getDrvDist(int Le_i_MtrBnk) { 
        double Le_r_DrvEncdrNormCnts = (double)Va_h_DrvMtr[Le_i_MtrBnk].getSelectedSensorPosition();
        return getDrvInchesPerEncdrCnts(Le_r_DrvEncdrNormCnts);
    }


	public double getSpdMult() {
		return VeSDRV_r_MtrSpdMult;
	}

	public void setSpdMult(double LeSDRV_r_MtrSpdMult) {
		this.VeSDRV_r_MtrSpdMult = LeSDRV_r_MtrSpdMult;
	}	

}



/*

        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            / * Print to SmartDashboard * /
            SmartDashboard.putNumber("Module Target Angle Caddy (Degrees) " + Me_i_ModIdx, Le_Deg_RotAngTgt);
            SmartDashboard.putNumber("Module Target Angle Caddy (Norm Rot) " + Me_i_ModIdx, Le_r_RotAngTgt);
            SmartDashboard.putNumber("Module Target Angle Motor (Norm Rot) " + Me_i_ModIdx, Le_r_RotAngTgtMtr);
        }


	    if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            / * Print to SmartDashboard * /
          SmartDashboard.putNumber("Module Actual Angle Front (Enc Counts) " + Me_i_ModIdx, Ms_h_RotEncdr.getPosition());
          SmartDashboard.putNumber("Module Actual Angle ReAR (Enc Counts) " + Me_i_ModIdx, Ms_h_RotEncdr.getPosition());
          SmartDashboard.putNumber("Module Actual Angle (Degrees)    " + Me_i_ModIdx, Le_Deg_RotAngAct);
          SmartDashboard.putNumber("Module Angle Error (Degrees)     " + Me_i_ModIdx, Le_Deg_RotErrAbs);
          SmartDashboard.putString("Module Driver Motor Init Direction  " + Me_i_ModIdx, Me_e_DrvMtrDirctn.toString());
        }

        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            / * Print to SmartDashboard * /
          SmartDashboard.putBoolean("Module Direction Update Trigger  " + Me_i_ModIdx, Me_b_DrvMtrDirctnUpdTrig);
          SmartDashboard.putString("Module Driver Motor Update  Direction    " + Me_i_ModIdx, Me_e_DrvMtrDirctn.toString());
        }
 */


