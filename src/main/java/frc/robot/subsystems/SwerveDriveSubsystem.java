package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.calibrations.K_SWRV;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.SPI;

public class SwerveDriveSubsystem extends HolonomicDrivetrainSubsystem {

	private static final double VeSDRV_r_ChassisTrkRat = Math.sqrt(Math.pow(K_SWRV.KeSWRV_l_ChassisWhlBase, 2) + Math.pow(K_SWRV.KeSWRV_l_ChassisTrkWdth, 2));

	private SwerveDriveModule[] SwrvDrvMod = new SwerveDriveModule[] {                            
		new SwerveDriveModule(SwrvMap.RtFt, new CANSparkMax(Constants.SWRV_FR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_RT_DRV), 0), //real:298 practice: 56
		new SwerveDriveModule(SwrvMap.LtFt, new CANSparkMax(Constants.SWRV_FR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_FR_LT_DRV), 0), //real:355 practice: 190
		new SwerveDriveModule(SwrvMap.LtRr, new CANSparkMax(Constants.SWRV_RR_LT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_LT_DRV), 0), //real:293 practice: 59
		new SwerveDriveModule(SwrvMap.RtRr, new CANSparkMax(Constants.SWRV_RR_RT_ROT, MotorType.kBrushless), new TalonFX(Constants.SWRV_RR_RT_DRV), 0)  //real:390 practice: 212
	};

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);


    boolean VeSDRV_b_SwrvRotRqstActv;

	double[] VaSDRV_Deg_RotAngCalcRaw = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtd = new double[SwrvMap.NumOfCaddies];
    double[] VaSDRV_Deg_RotAngCalcLtchd = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotAngCalcTgt = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_v_DrvSpdCalcRaw = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_v_DrvSpdCalcCnvrtd = new double[SwrvMap.NumOfCaddies];

	private double VeSDRV_v_DrvSpdMax = 0.0;

	private double VeSDRV_r_MtrSpdMult = 1.0;


	public SwerveDriveSubsystem() {
		zeroGyro(); 

		SwrvDrvMod[SwrvMap.RtFt].getDrvMtr().setInverted(true);  //real: true
		SwrvDrvMod[SwrvMap.LtFt].getDrvMtr().setInverted(false); //real: false
		SwrvDrvMod[SwrvMap.LtRr].getDrvMtr().setInverted(false); //real: false
		SwrvDrvMod[SwrvMap.RtRr].getDrvMtr().setInverted(true);  //real: true

		SwrvDrvMod[0].resetDrvZeroPstn();

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
			SwrvDrvMod[i].getDrvMtr().setNeutralMode(NeutralMode.Brake);
		}
		
	}

	public AHRS getNavX() {
		return mNavX;
	}

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

	public SwerveDriveModule getSwerveModule(int i) {
		return SwrvDrvMod[i];
	}


	@Override
	public void HolonomicDrv(double forward, double strafe, double rotation) {
		
    VeSDRV_b_SwrvRotRqstActv = Math.abs(rotation) > .1;

		forward *= getSpdMult();
		strafe *= getSpdMult();
		/*
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = forward * Math.cos(angleRad) + strafe * Math.sin(angleRad);
			strafe = -forward * Math.sin(angleRad) + strafe * Math.cos(angleRad);
			forward = temp;
		}
		*/

		double a = strafe - rotation * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double b = strafe + rotation * (K_SWRV.KeSWRV_l_ChassisWhlBase / K_SWRV.KeSWRV_l_ChassisTrkWdth);
		double c = forward - rotation * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);
		double d = forward + rotation * (K_SWRV.KeSWRV_l_ChassisTrkWdth / K_SWRV.KeSWRV_l_ChassisWhlBase);

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
			if (Math.abs(forward) > 0.05 || Math.abs(strafe) > 0.05 || Math.abs(rotation) > 0.05) {
				// Save Away Target Angle for Feedback (Latched Values) if no update to target drive;
        VaSDRV_Deg_RotAngCalcLtchd[i] = VaSDRV_Deg_RotAngCalcCnvrtd[i];	
				SwrvDrvMod[i].setRotTgtAng(VaSDRV_Deg_RotAngCalcCnvrtd[i]);
			} else {
				SwrvDrvMod[i].setRotTgtAng(VaSDRV_Deg_RotAngCalcLtchd[i]);
			}

			SwrvDrvMod[i].dtrmnDrvMtrDirctn(VaSDRV_Deg_RotAngCalcLtchd[i], VeSDRV_b_SwrvRotRqstActv);

			
			SwrvDrvMod[i].setDrvMtrSpd(VaSDRV_v_DrvSpdCalcRaw[i]);
		}
	}

	@Override
	public void stopDrvMtrs() {
		for (SwerveDriveModule module : SwrvDrvMod) {
			module.setDrvMtrSpd(0);
		}
	}

	public void resetDrvEncdrs() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++ ) {
			SwrvDrvMod[i].resetDrvZeroPstn();
		}
	}

	public void resetRotEncdrs() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].resetRotEncdr();
		}
	}

	public void driveForwardDistance(double targetPos, double angle, double speed){
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;

		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		HolonomicDrv(speed, 0, 0);
	}

	public void driveSidewaysDistance(double targetPos, double angle, double speed) {
		double angleError = ((angle - mNavX.getYaw()) / 180)*10;
		angleError = Math.min(angleError, 1);
		angleError = Math.max(angleError, -1);
		HolonomicDrv(0, speed, angleError);
	}

	public double calcDrvPosErr(double d1) {
		return d1 - SwrvDrvMod[0].getDrvDist();
	}

	public void cmndDrvToEncdrPos(double Le_r_EncdrPos) {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].setRotEncdrTgtPstn(0);
		}
	}

	public double getSpdMult() {
		return VeSDRV_r_MtrSpdMult;
	}

	public void setSpdMult(double LeSDRV_r_MtrSpdMult) {
		this.VeSDRV_r_MtrSpdMult = LeSDRV_r_MtrSpdMult;
	}	

}
