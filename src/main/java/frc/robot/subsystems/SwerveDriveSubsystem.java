package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.Constants.DriveShiftPos;
import frc.robot.Constants.SolenoidPosition;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;
import frc.robot.subsystems.SwerveDriveModule.TeRotDirctn;
import frc.robot.subsystems.SwerveDriveModule.Te_RZL_St;

import com.kauailabs.navx.frc.AHRS;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import org.w3c.dom.ls.LSException;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpiutil.math.MathUtil;
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

	private PIDController VsSDRV_PID_Long;
    private PIDController VsSDRV_PID_Lat;
    private PIDController VsSDRV_PID_Rot;

    private double VeSDRV_r_PwrOutputMinLong = -1;
    private double VeSDRV_r_PwrOutputMaxLong =  1;
    private double VeSDRV_r_PwrOutputMinLat  = -1;
    private double VeSDRV_r_PwrOutputMaxLat  =  1;
    private double VeSDRV_r_PwrOutputMinRot  = -1;
    private double VeSDRV_r_PwrOutputMaxRot  =  1;

	private boolean VeSDRV_b_SwrvRotRqstActv;
	private boolean VeSDRV_b_RZL_Rqst;
	private boolean VeSDRV_b_RZL_Actv;
	private boolean VeSDRV_b_RZL_Cmplt;
	private boolean VeSDRV_b_RZL_FailIntrsv;
	private Timer   VeSDRV_t_RZL_TmeOutTmrIntrsv = new Timer();
	

	double[] VaSDRV_r_RotEncdrActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrActCorr    = new double[SwrvMap.NumOfCaddies];	
	double[] VaSDRV_Deg_RotAngActRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngActNorm    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngActCnvrtd  = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngCalcRaw    = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcCnvrtd = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngCalcLtch   = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_Deg_RotAngTgtRaw     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_Deg_RotAngTgtMod     = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_r_RotEncdrTgt        = new double[SwrvMap.NumOfCaddies];      
    double[] VaSDRV_r_RotEncdrTgtCorr    = new double[SwrvMap.NumOfCaddies];

	double[] VaSDRV_v_DrvSpdCalcRaw      = new double[SwrvMap.NumOfCaddies];
	double[] VaSDRV_v_DrvSpdCalcCnvrtd   = new double[SwrvMap.NumOfCaddies];

	
	private double VeSDRV_v_DrvSpdMax;

	private double VeSDRV_r_MtrSpdMult;

	private double VeSDRV_r_PwrLong;
	private double VeSDRV_r_PwrLat;
	private double VeSDRV_r_PwrRot;

  /** SwrvRotZeroNotRun: While True indicates that the Swerve Rotation Angle Zero
    * Learn has not completed.
    */
	private boolean VeSDRV_b_RotZeroLearnNotRun;

	private int     VeTEST_Cnt_TestBrnchCntr;
	private boolean VeTEST_b_TestInitTrig;

	/*******************************/
	/* Subsystem Constructor       */
	/*******************************/
	public SwerveDriveSubsystem() {
		zeroGyro(); 

		SwrvDrvMod[SwrvMap.LtRr].getDrvMtr().follow(SwrvDrvMod[SwrvMap.LtFt].getDrvMtr());
		SwrvDrvMod[SwrvMap.RtRr].getDrvMtr().follow(SwrvDrvMod[SwrvMap.RtFt].getDrvMtr());

		VsSDRV_PID_Long = new PIDController(K_SWRV.KeSWRV_k_CL_PropGx_Long, K_SWRV.KeSWRV_k_CL_IntglGx_Long, K_SWRV.KeSWRV_k_CL_DerivGx_Long);
		VsSDRV_PID_Lat  = new PIDController(K_SWRV.KeSWRV_k_CL_PropGx_Lat,  K_SWRV.KeSWRV_k_CL_IntglGx_Lat,  K_SWRV.KeSWRV_k_CL_DerivGx_Lat);
		VsSDRV_PID_Rot  = new PIDController(K_SWRV.KeSWRV_k_CL_PropGx_Rot,  K_SWRV.KeSWRV_k_CL_IntglGx_Rot,  K_SWRV.KeSWRV_k_CL_DerivGx_Rot);

		VeSDRV_r_PwrOutputMinLong = -1;
		VeSDRV_r_PwrOutputMaxLong =  1;
		VeSDRV_r_PwrOutputMinLat  = -1;
		VeSDRV_r_PwrOutputMaxLat  =  1;
		VeSDRV_r_PwrOutputMinRot  = -1;
		VeSDRV_r_PwrOutputMaxRot  =  1;
	

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
		  VaSDRV_r_RotEncdrActRaw[i]    = 0;
		  VaSDRV_r_RotEncdrActCorr[i]   = 0;
		  VaSDRV_Deg_RotAngActRaw[i]    = 0;
		  VaSDRV_Deg_RotAngActNorm[i]   = 0;
		  VaSDRV_Deg_RotAngActCnvrtd[i] = 0;
		  VaSDRV_Deg_RotAngCalcLtch[i]  = 0;
		  VaSDRV_v_DrvSpdCalcRaw[i]     = 0;
		  VaSDRV_v_DrvSpdCalcCnvrtd[i]  = 0;
		  VaSDRV_Deg_RotAngTgtRaw[i]    = 0;
		  VaSDRV_Deg_RotAngTgtMod[i]    = 0;
		  VaSDRV_r_RotEncdrTgt[i]       = 0;
		  VaSDRV_r_RotEncdrTgtCorr[i]   = 0;
		}

	    VeSDRV_b_SwrvRotRqstActv = false;
		VeSDRV_b_RZL_Rqst        = false;
		VeSDRV_b_RZL_Actv        = false;
		VeSDRV_b_RZL_Cmplt       = false;
		VeSDRV_b_RZL_FailIntrsv  = false;
		VeSDRV_t_RZL_TmeOutTmrIntrsv.reset();

		VeSDRV_v_DrvSpdMax  = 0.0;
		VeSDRV_r_MtrSpdMult = 1.0;

		VeSDRV_r_PwrLong = 0.0;
		VeSDRV_r_PwrLat  = 0.0;
		VeSDRV_r_PwrRot  = 0.0;

		VeSDRV_b_RotZeroLearnNotRun = false;

		VeTEST_Cnt_TestBrnchCntr = 0;
		VeTEST_b_TestInitTrig = false;
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


	public void resetGyro() {
		mNavX.reset();
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




    /*********************************************/
	/*     Subsystem Method Definitions          */
	/*********************************************/

	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	  /* Update SmartDashboard with Data */

	  VeTEST_Cnt_TestBrnchCntr = 1;
	  mngSDRV_RZL_PeriodicIntrsvTask();
	  mngSDRV_RZL_PeriodicPassiveTask();

	  if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
	    updateSmartDash();	
	  }
	
	}



    public void mngSDRV_RZL_PeriodicIntrsvTask() {
		boolean Le_b_RZL_InitTrig = false;
		Te_RZL_St Le_e_St_RtFt, Le_e_St_LtFt, Le_e_St_LtRr, Le_e_St_RtRr;
	   
		  if (VeSDRV_b_RZL_Rqst == true) {
			if (VeSDRV_b_RZL_Actv == false) {
			  VeSDRV_t_RZL_TmeOutTmrIntrsv.reset();
			  VeSDRV_t_RZL_TmeOutTmrIntrsv.start();
			  VeTEST_Cnt_TestBrnchCntr = 1;
			  VeTEST_b_TestInitTrig = true;	
			  Le_b_RZL_InitTrig = true;
			  VeSDRV_b_RZL_Actv = true;
			}
  
		  setDrvMtrSpdsZero();
		  VeTEST_Cnt_TestBrnchCntr = 2;	

		  Le_e_St_RtFt = SwrvDrvMod[SwrvMap.RtFt].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		  Le_e_St_LtFt = SwrvDrvMod[SwrvMap.LtFt].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		  Le_e_St_LtRr = SwrvDrvMod[SwrvMap.LtRr].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);
		  Le_e_St_RtRr = SwrvDrvMod[SwrvMap.RtRr].learnRotEncdrZeroOfst(Le_b_RZL_InitTrig);			
  
		  if ((Le_e_St_RtFt == Te_RZL_St.Cmplt) && (Le_e_St_LtFt == Te_RZL_St.Cmplt) &&
			  (Le_e_St_LtRr == Te_RZL_St.Cmplt) && (Le_e_St_RtRr == Te_RZL_St.Cmplt)) {
			VeSDRV_b_RZL_Cmplt = true;
			VeTEST_Cnt_TestBrnchCntr = 3;
		  }
  
		  if (VeSDRV_t_RZL_TmeOutTmrIntrsv.get() >= K_SWRV.KeSWRV_t_RZL_TimeOutThrshIntrsv) {
			VeSDRV_b_RZL_Cmplt = true;
			VeSDRV_b_RZL_FailIntrsv = true;
			VeTEST_Cnt_TestBrnchCntr = 4;
		  }

		}
		else /*  (VeSDRV_b_RZL_Rqst == false) */ {
		  VeSDRV_t_RZL_TmeOutTmrIntrsv.stop();
		  VeSDRV_t_RZL_TmeOutTmrIntrsv.reset();
		  VeSDRV_b_RZL_Actv = false;	
		  VeTEST_Cnt_TestBrnchCntr = 5;	
		  SwrvDrvMod[SwrvMap.RtFt].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
		  SwrvDrvMod[SwrvMap.LtFt].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
		  SwrvDrvMod[SwrvMap.LtRr].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
		  SwrvDrvMod[SwrvMap.RtRr].setRotEncdrZeroLearnSt(Te_RZL_St.Inactv);
		}
  	  }


	public void mngSDRV_RZL_PeriodicPassiveTask() {
	  for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		SwrvDrvMod[i].montrRotEncdrZeroOfst();
	  }		
	}



    /**
     * Control Swerve Drive with 3 PID controllers, one for each of the axes.
     * @param Le_x_CL_ErrLong The error term (from the input device) for forward movement.
     * @param Le_x_CL_ErrLat The error term (from the input device) for strafe movement.
     * @param Le_x_CL_ErrRot The error term (from the input device, generally gyroscope) for rotation.
     * @param fieldOriented Whether the frame of reference for Forward and Strafe is the field (true) or the robot (false).
     */
    public void PID_DrvCntrl(double Le_x_CL_ErrLong, double Le_x_CL_ErrLat, double Le_x_CL_ErrRot, boolean Le_b_FieldOriented){
		double forward = VsSDRV_PID_Long.calculate(Le_x_CL_ErrLong);
		double strafe = VsSDRV_PID_Lat.calculate(Le_x_CL_ErrLat);
		double rotation = VsSDRV_PID_Rot.calculate(Le_x_CL_ErrRot);
  
		forward = MathUtil.clamp(forward, VeSDRV_r_PwrOutputMinLong, VeSDRV_r_PwrOutputMaxLong);
		strafe = MathUtil.clamp(strafe, VeSDRV_r_PwrOutputMinLat, VeSDRV_r_PwrOutputMaxLat);
		rotation = MathUtil.clamp(rotation, VeSDRV_r_PwrOutputMinRot, VeSDRV_r_PwrOutputMaxRot);
  
		HolonomicDrv(forward, strafe, rotation, Le_b_FieldOriented);
	  }
  



	@Override
	public void HolonomicDrv(double Le_r_PwrLong, double Le_r_PwrLat, double Le_r_PwrRot, boolean Le_b_FieldOriented) {
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
		
		if (isFieldOriented()) {
			double angleRad = Math.toRadians(getGyroAngle());
			double temp = VeSDRV_r_PwrLong * Math.cos(angleRad) + VeSDRV_r_PwrLat * Math.sin(angleRad); 
			VeSDRV_r_PwrLat = -VeSDRV_r_PwrLong * Math.sin(angleRad) + VeSDRV_r_PwrLat * Math.cos(angleRad);
			VeSDRV_r_PwrLong = temp;
		}


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
		  /* Get Rotation Encoder Position Raw */
		  VaSDRV_r_RotEncdrActRaw[i] = SwrvDrvMod[i].getRotEncdrActPstn();
		  /* Apply Rotation Encoder Position Zero Offset Correction to Raw Position */
          VaSDRV_r_RotEncdrActCorr[i] = SwrvDrvMod[i].correctRotEncdrActPstn(VaSDRV_r_RotEncdrActRaw[i]); 
		  /* Calculate Caddy Angle from Encoder Position (can include multiple revolutions) */         
		  VaSDRV_Deg_RotAngActRaw[i] = SwrvDrvMod[i].calcRotActAng(VaSDRV_r_RotEncdrActCorr[i]);
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
			VaSDRV_Deg_RotAngTgtRaw[i] = SwrvDrvMod[i].cnvrtRotTgtAng(VaSDRV_Deg_RotAngCalcLtch[i]);
		}
		VaSDRV_Deg_RotAngTgtMod = calcRotAngTgtOptmzd(VaSDRV_Deg_RotAngTgtRaw);
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			VaSDRV_r_RotEncdrTgt[i] = SwrvDrvMod[i].calcRotEncdrTgt(VaSDRV_Deg_RotAngTgtMod[i]);
			VaSDRV_r_RotEncdrTgtCorr[i] = SwrvDrvMod[i].correctRotEncdrTgtPstn(VaSDRV_r_RotEncdrTgt[i]);
			SwrvDrvMod[i].setRotEncdrTgt(VaSDRV_r_RotEncdrTgt[i]);
		}

		/* Command Target Drive Speeds */
		setDrvMtrSpdByBank(SwrvMap.RtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.RtFt]);
		setDrvMtrSpdByBank(SwrvMap.LtSd, VaSDRV_v_DrvSpdCalcCnvrtd[SwrvMap.LtFt]);
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


	/** Method: calcRotAngTgtOptmzd - Swerve Drive System: Determines the
	 * Rotation Angle Target for the Swerve Modules to reduce the amount of
	 * rotation requied and to coordinate the direction of rotation of the
	 * coupled Swerve Modules.
	 * @param  La_Deg_AngTgtRaw  (double[]: Swerve Module Rotation Target Angle Raw )
	 * @return La_Deg_AngTgtMod  (double[]: Swerve Module Rotation Target Angle Modified )
     */   
    private double[] calcRotAngTgtOptmzd(double[] La_Deg_AngTgtRaw) {
		double[] Ls_Deg_AngTgtTemp = new double[SwrvMap.NumOfCaddies];
		double[] Ls_Deg_AngErr    = new double[SwrvMap.NumOfCaddies];
		double[] La_Deg_AngTgtMod = new double[SwrvMap.NumOfCaddies];
		TeRotDirctn Le_e_RotDirctnLt , Le_e_RotDirctnRt; 

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			Ls_Deg_AngTgtTemp[i] = (SwrvDrvMod[i].getRotModRevs() * 360) + VaSDRV_Deg_RotAngActRaw[i];
			Ls_Deg_AngErr[i] = Ls_Deg_AngTgtTemp[i] -  VaSDRV_Deg_RotAngActRaw[i]; 
		}

		Le_e_RotDirctnLt = dtrmnRotAngRotDirctn(SwrvDrvMod[SwrvMap.LtFt].getDrvMtrDirctn(), SwrvDrvMod[SwrvMap.LtRr].getDrvMtrDirctn(),
			                                    Ls_Deg_AngErr[SwrvMap.LtFt], Ls_Deg_AngErr[SwrvMap.LtRr],
												VaSDRV_Deg_RotAngActRaw[SwrvMap.LtFt], VaSDRV_Deg_RotAngActRaw[SwrvMap.LtFt],
												SwrvDrvMod[SwrvMap.LtFt].getRotModRevs(), SwrvDrvMod[SwrvMap.LtFt].getRotModRevs());
		Le_e_RotDirctnRt = dtrmnRotAngRotDirctn(SwrvDrvMod[SwrvMap.RtFt].getDrvMtrDirctn(), SwrvDrvMod[SwrvMap.RtRr].getDrvMtrDirctn(),
		                                        Ls_Deg_AngErr[SwrvMap.RtFt], Ls_Deg_AngErr[SwrvMap.RtRr],
		                                        VaSDRV_Deg_RotAngActRaw[SwrvMap.RtFt], VaSDRV_Deg_RotAngActRaw[SwrvMap.RtFt],
		                                        SwrvDrvMod[SwrvMap.RtFt].getRotModRevs(), SwrvDrvMod[SwrvMap.RtFt].getRotModRevs());

	    return(La_Deg_AngTgtMod);
    }


	private TeRotDirctn dtrmnRotAngRotDirctn(TeMtrDirctn Le_e_DrvMtrDirctnFt , TeMtrDirctn Le_e_DrvMtrDirctnRr,
	                                         double Le_Deg_AngErrFt,           double Le_Deg_AngErrRr,
											 double Le_Deg_AngActRawFt,        double Le_Deg_AngActRawRr,
											 int Le_Cnt_RotModRevsFt,          int Le_Cnt_RotModRevsRr) {
		TeRotDirctn	Le_e_RotDirctn = TeRotDirctn.CW;
		// todo - this logic needs to be worked out, how do determine proper rotation to prevent binding 
	    if (VeSDRV_r_PwrRot > 0) {
			if ((Le_Deg_AngActRawFt > 0) && (Le_Deg_AngErrFt > 0)) {
				if (Le_e_DrvMtrDirctnFt == TeMtrDirctn.Rwd){

				}
				Le_e_RotDirctn = TeRotDirctn.CW;
			}
			Le_e_RotDirctn = TeRotDirctn.CCW;
		}
		else {

		}
		return(Le_e_RotDirctn);
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
		HolonomicDrv(Le_r_PwrLvl, 0, 0, false);
	}


	public void driveForwardToDist(double Le_l_TgtPos, double Le_Deg_HdgAng, double Le_r_PwrLvl){
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(Le_r_PwrLvl, 0, Le_Deg_HdgAngErr, false);
	}


	public void driveSidewaysAtSpd(double Le_Deg_HdgAng, double Le_r_PwrLvl) {
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(0, Le_r_PwrLvl, 0, false);
	}


	public void driveSidewaysToDist(double Le_l_TgtPos, double Le_Deg_HdgAng, double Le_r_PwrLvl) {
		double Le_Deg_HdgAngErr = ((Le_Deg_HdgAng - mNavX.getYaw()) / 180)*10;
		Le_Deg_HdgAngErr = Math.min(Le_Deg_HdgAngErr, 1);
		Le_Deg_HdgAngErr = Math.max(Le_Deg_HdgAngErr, -1);
		HolonomicDrv(0, Le_r_PwrLvl, Le_Deg_HdgAngErr, false);
	}



	public void driveRotateAtSpd(TeRotDirctn Le_e_RotDirctn, double Le_r_PwrLvl) {
        double Le_r_RotPwr;
		if (Le_e_RotDirctn == TeRotDirctn.CCW)
		  Le_r_RotPwr = Le_r_PwrLvl * -1;
		else
		  Le_r_RotPwr = Le_r_PwrLvl;
		Le_r_RotPwr = Math.min(Le_r_RotPwr, 1);
		Le_r_RotPwr = Math.max(Le_r_RotPwr, -1);
		HolonomicDrv(0, 0, Le_r_RotPwr, false);
	}


	public void driveRotateToAng(TeRotDirctn Le_e_RotDirctn, double Le_Deg_RotTgt, double Le_r_PwrLvl) {
		double Le_Deg_RotErr = ((Le_Deg_RotTgt - mNavX.getYaw()) / 180)*10;
		Le_Deg_RotErr = Math.min(Le_Deg_RotErr, 1);
		Le_Deg_RotErr = Math.max(Le_Deg_RotErr, -1);
		if (Math.abs(Le_Deg_RotErr) < 5 )
		  Le_r_PwrLvl *= 0.25;
		HolonomicDrv(0, 0, Le_r_PwrLvl, false);
	}


	public void cntrlBrkInSwrvDrv(double Le_r_PwrRot, double Le_r_PwrDrv) {	
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
       	   SwrvDrvMod[i].setRotMtrPwr(Le_r_PwrRot);
		}
		SwrvDrvMod[SwrvMap.RtFt].setDrvMtrPwr(Le_r_PwrDrv);
		SwrvDrvMod[SwrvMap.LtFt].setDrvMtrPwr(Le_r_PwrDrv);
	}


	
    /*************************************************/
	/*     Subsystem Data Interfaces                 */
	/*************************************************/


	public int getDrvCaddyEncdrCnts() {
		return((int) Math.round(SwrvDrvMod[SwrvMap.RtFt].getDrvEncdrCntsRel()));
    }
  
	public double getDrvCaddyInchesPerEncdrCnts(int Le_Cnts_DrvEncdrCnts, DriveShiftPos Le_e_DrvSlctdGr) {
	  double Le_l_DrvWhlDistInches;
	  Le_l_DrvWhlDistInches = SwrvDrvMod[SwrvMap.RtFt].getDrvInchesPerEncdrCnts(Le_Cnts_DrvEncdrCnts, Le_e_DrvSlctdGr); 
	  return(Le_l_DrvWhlDistInches);
	}

	public int getDrvCaddyEncdrCntsPerInches(double Le_l_DrvWhlDistInches, DriveShiftPos Le_e_DrvSlctdGr) {
	  int Le_Cnts_DrvEncdr;
	  Le_Cnts_DrvEncdr = SwrvDrvMod[SwrvMap.RtFt].getDrvEncdrCntsPerInches(Le_l_DrvWhlDistInches, Le_e_DrvSlctdGr);
	  return(Le_Cnts_DrvEncdr);  
	}
	

	public double getDrvCaddyDistTravelled(DriveShiftPos Le_e_DrvSlctdGr) {
	  double Le_l_DrvWhlDistInches;
	  Le_l_DrvWhlDistInches =  SwrvDrvMod[SwrvMap.RtFt].getDrvDistTravelled(Le_e_DrvSlctdGr);
	  return(Le_l_DrvWhlDistInches);
	}


	public double calcDrvCaddyDistErr(double Le_l_DrvDistTgtInches, DriveShiftPos Le_e_DrvSlctdGr) {
		return(Le_l_DrvDistTgtInches - SwrvDrvMod[SwrvMap.RtFt].getDrvDistTravelled(Le_e_DrvSlctdGr));
	}


	public void cmndDrvsToEncdrPos(double Le_r_EncdrPos) {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
			SwrvDrvMod[i].setRotEncdrTgt(Le_r_EncdrPos);
		}
	}


	public void haltSwerveDrive(){
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
			SwrvDrvMod[i].setRotMtrPwr(0);
            SwrvDrvMod[i].setDrvMtrPwr(0);
		}
	}

	@Override
	public void stopDrvMtrs() {
		for (int i = 0; i < SwrvMap.NumOfBanks; i++ ) {
			setDrvMtrSpdByBank(i, 0);
		}
	}

	public void setDrvMtrSpdsZero() {
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
		  SwrvDrvMod[i].setDrvMtrPwr(0);
		  SwrvDrvMod[i].setDrvMtrPwr(0);
		  SwrvDrvMod[i].setDrvMtrPwr(0);
		  SwrvDrvMod[i].setDrvMtrPwr(0);
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


	public double getSwerveCaddyAng(int Le_i_SwrvMod) {
	    return(SwrvDrvMod[Le_i_SwrvMod].getRotAngActRaw());
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


	public boolean getRotZeroDtctd(int Le_i_SwrvModIdx) {
	     return(SwrvDrvMod[Le_i_SwrvModIdx].getRotEncdrZeroDtctd());
	}



    public void setDrvMtrSpdByBank(int Le_i_MtrBnk, double Le_r_DrvPwr) {
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
	/*  Drive PID Methods for configuring PID        */
	/*************************************************/
	// All the PID System methods. 
	
    public void setForwardSetpoint(double setpoint){
		VsSDRV_PID_Long.setSetpoint(setpoint);
	  }
	  public void setStrafeSetpoint(double setpoint){
		VsSDRV_PID_Lat.setSetpoint(setpoint);
	  }
	  public void setRotationSetpoint(double setpoint){
		VsSDRV_PID_Rot.setSetpoint(setpoint);
	  }
	  public void setForwardTolerance(double positionTolerance, double velocityTolerance){
		VsSDRV_PID_Long.setTolerance(positionTolerance, velocityTolerance);
	  }
	  public void setStrafeTolerance(double positionTolerance, double velocityTolerance){
		VsSDRV_PID_Lat.setTolerance(positionTolerance, velocityTolerance);
	  }
	  public void setRotationTolerance(double positionTolerance, double velocityTolerance){
		VsSDRV_PID_Rot.setTolerance(positionTolerance, velocityTolerance);
	  }
	  public void setForwardWraparoundInputRange(double min, double max){
		VsSDRV_PID_Long.enableContinuousInput(min, max);
	  }
	  public void setStrafeWraparoundInputRange(double min, double max){
		VsSDRV_PID_Lat.enableContinuousInput(min, max);
	  }
	  public void setRotationWraparoundInputRange(double min, double max){
		VsSDRV_PID_Rot.enableContinuousInput(min, max);
	  }
	  public void setForwardAccumulationRange(double min, double max){
		VsSDRV_PID_Long.setIntegratorRange(min, max);
	  }
	  public void setStrafeAccumulationRange(double min, double max){
		VsSDRV_PID_Lat.setIntegratorRange(min, max);
	  }
	  public void setRotationAccumulationRange(double min, double max){
		VsSDRV_PID_Rot.setIntegratorRange(min, max);
	  } 
	  public void setForwardOutputRange(double minOutput, double maxOutput){
		VeSDRV_r_PwrOutputMinLong = minOutput;
		VeSDRV_r_PwrOutputMaxLong = maxOutput;
	  }
	  public void setStrafeOutputRange(double minOutput, double maxOutput){
		VeSDRV_r_PwrOutputMinLat = minOutput;
		VeSDRV_r_PwrOutputMaxLat = maxOutput;
	  }
	  public void setRotationOutputRange(double minOutput, double maxOutput) {
		VeSDRV_r_PwrOutputMinRot = minOutput;
		VeSDRV_r_PwrOutputMaxRot = maxOutput;
	  }
	  public double getForwardErrorDerivative(){
		return VsSDRV_PID_Long.getVelocityError();
	  }
	  public double getStrafeErrorDerivative(){
		return VsSDRV_PID_Lat.getVelocityError();
	  }
	  public double getRotationErrorDerivative(){
		return VsSDRV_PID_Rot.getVelocityError();
	  }
	  public boolean forwardAtSetpoint(){
		return VsSDRV_PID_Long.atSetpoint();
	  }
	  public boolean strafeAtSetpoint(){
		return VsSDRV_PID_Lat.atSetpoint();
	  }
	  public boolean rotationAtSetpoint(){
		return VsSDRV_PID_Rot.atSetpoint();
	  }
	  /**
	   * Clear all I accumulation, disable continuous input, and set all 
	   * setpoints to 0.
	   */
	  public void resetPID(){
		// clear I accumulation
		VsSDRV_PID_Long.reset();
		VsSDRV_PID_Lat.reset();
		VsSDRV_PID_Rot.reset();
  
		// reset to noncontinuous input
		VsSDRV_PID_Long.disableContinuousInput();
		VsSDRV_PID_Lat.disableContinuousInput();
		VsSDRV_PID_Rot.disableContinuousInput();
  
		// set all setpoints to 0
		VsSDRV_PID_Long.setSetpoint(0);
		VsSDRV_PID_Lat.setSetpoint(0);
		VsSDRV_PID_Rot.setSetpoint(0);
  
		// set all I accumulation ranges to defaults
		VsSDRV_PID_Long.setIntegratorRange(-1.0, 1.0);
		VsSDRV_PID_Lat.setIntegratorRange(-1.0,1.0);
		VsSDRV_PID_Rot.setIntegratorRange(-1.0, 1.0);
  
		VsSDRV_PID_Long.setTolerance(0.05, Double.POSITIVE_INFINITY);
		VsSDRV_PID_Lat.setTolerance(0.05, Double.POSITIVE_INFINITY);
		VsSDRV_PID_Rot.setTolerance(0.05, Double.POSITIVE_INFINITY);
  
		VeSDRV_r_PwrOutputMinLong = -1;
		VeSDRV_r_PwrOutputMaxLong = 1;
		VeSDRV_r_PwrOutputMinLat = -1;
		VeSDRV_r_PwrOutputMaxLat = 1;
		VeSDRV_r_PwrOutputMinRot = -1;
		VeSDRV_r_PwrOutputMaxRot = 1;
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
		SmartDashboard.putBoolean("RZL Actv " , VeSDRV_b_RZL_Actv);
		SmartDashboard.putBoolean("RZL Cmplt " , VeSDRV_b_RZL_Cmplt);
		SmartDashboard.putBoolean("RZL Fail Intrsv " , VeSDRV_b_RZL_FailIntrsv);

		SmartDashboard.putNumber("Test Branch " , VeTEST_Cnt_TestBrnchCntr);
		SmartDashboard.putBoolean("Test Init ", VeTEST_b_TestInitTrig);

		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
	
		   SmartDashboard.putString("RZL St " + i ,            SwrvDrvMod[i].getRotEncdrZeroLearnSt().toString());
		   SmartDashboard.putNumber("RZL Cntr " + i ,          SwrvDrvMod[i].getRotEncdrZeroLearnCntr());
	
		   SmartDashboard.putNumber("Calc Ang Raw " + i ,      VaSDRV_Deg_RotAngCalcRaw[i]);
		   SmartDashboard.putNumber("Calc Ang Cnvrtd " + i ,   VaSDRV_Deg_RotAngCalcCnvrtd[i]);
		   SmartDashboard.putNumber("Calc Ang Ltch " + i ,     VaSDRV_Deg_RotAngCalcLtch[i]);
	
		   SmartDashboard.putNumber("Tgt Ang Raw " + i ,       VaSDRV_Deg_RotAngTgtRaw[i]);
		   SmartDashboard.putNumber("Tgt Ang Mod " + i ,       VaSDRV_Deg_RotAngTgtMod[i]);		   	   
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
		   SmartDashboard.putNumber("Drv Encdr Cnts " + i ,    SwrvDrvMod[i].getDrvEncdrCntsRel());	   
		   SmartDashboard.putString("Drv Mtr Dir " + i ,       SwrvDrvMod[i].getDrvMtrDirctn().toString());
		   SmartDashboard.putBoolean("Drv Mtr Dir Trig " + i , SwrvDrvMod[i].getDrvMtrDirctnTrig());
		   
		}
	
	}
	



}

