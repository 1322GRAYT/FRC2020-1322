/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpiutil.math.MathUtil;

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;
import frc.robot.DrvMap;
import frc.robot.calibrations.K_DRV;
import frc.robot.subsystems.RFSLIB;

/**
 * Add your docs here.
 */
public class DriveSubsystem extends SubsystemBase {
	private RFSLIB rfsLIB = new RFSLIB();

	private AHRS mNavX = new AHRS(SPI.Port.kMXP, (byte) 200);
	private double VeDRV_Deg_NAV_SnsdAng;
	private double VeDRV_Deg_NAV_DsrdAng;

    private TalonFX[] DriveMotor = new TalonFX[] { 
		new TalonFX(Constants.DRV_LT_FR),
		new TalonFX(Constants.DRV_LT_RR),
		new TalonFX(Constants.DRV_RT_FR),
		new TalonFX(Constants.DRV_RT_RR)
	};

	private boolean[] VaDRV_b_MtrInvrtd      = new boolean[DrvMap.NumOfMtrs];
	private int[] VaDRV_Cnt_DrvEncdrPstn     = new int[DrvMap.NumOfMtrs];
	private int[] VaDRV_Cnt_DrvEncdrZeroPstn = new int[DrvMap.NumOfMtrs];


    private double VeDRV_r_NormPwrRqstFwd;
	private double VeDRV_r_NormPwrRqstRot;
	private double VeDRV_r_NormPwrHdgCorr;

    private double VeDRV_r_NormPwrCmndLt;
    private double VeDRV_r_NormPwrCmndRt;

	/* Drive Control PID*/
	private PIDController VsDRV_PID_Drv;
	private double VeDRV_r_PID_DrvPowCorr;
    private double VeDRV_r_PID_DrvPwrOutMin;
    private double VeDRV_r_PID_DrvPwrOutMax;
    private double VeDRV_Cnt_PID_DrvPstnCmnd;

	/* Rotation Control PID*/
	private PIDController VsDRV_PID_Rot;
	private double VeDRV_r_PID_RotPowCorr;
    private double VeDRV_r_PID_RotPwrOutMin;
    private double VeDRV_r_PID_RotPwrOutMax;
    private double VeDRV_Deg_PID_RotAngCmnd;



	/*******************************/
	/* Subsystem Constructor       */
    /*******************************/
	public DriveSubsystem() {

		/* Gyro Initialization */
		zeroGyro();
		captureGyroAngRaw(); 
        VeDRV_Deg_NAV_DsrdAng =  VeDRV_Deg_NAV_SnsdAng;

		/* X-Box Controller and Drive Power Initialization */
        VeDRV_r_NormPwrRqstFwd = 0;
		VeDRV_r_NormPwrRqstRot = 0;
		VeDRV_r_NormPwrHdgCorr = 0;

        VeDRV_r_NormPwrCmndLt  = 0;
        VeDRV_r_NormPwrCmndRt  = 0;    

	    /* Drive Control PID*/
	    VsDRV_PID_Drv = new PIDController(K_DRV.KeDRV_k_CL_DrvPropGxFwd,  K_DRV.KeDRV_k_CL_DrvIntglGxFwd,  K_DRV.KeDRV_k_CL_DrvDerivGxFwd);
	    VeDRV_r_PID_DrvPowCorr    =  0;
        VeDRV_r_PID_DrvPwrOutMin  = -1;
        VeDRV_r_PID_DrvPwrOutMax  =  1;
        VeDRV_Cnt_PID_DrvPstnCmnd =  0;

	    /* Rotation Control PID*/
		VsDRV_PID_Rot  = new PIDController(K_DRV.KeDRV_k_CL_RotPropGx,  K_DRV.KeDRV_k_CL_RotIntglGx,  K_DRV.KeDRV_k_CL_RotDerivGx);
        VeDRV_r_PID_RotPowCorr    =  0;
		VeDRV_r_PID_RotPwrOutMin  = -1;
		VeDRV_r_PID_RotPwrOutMax  =  1;
		VeDRV_Deg_PID_RotAngCmnd  =  VeDRV_Deg_NAV_DsrdAng;


        /*****************************************************************/
        /* Drive Controller Configurations                               */
		/*****************************************************************/
        for (int i = 0; i < DrvMap.NumOfMtrs; i++) {
			DriveMotor[i].configFactoryDefault();
            DriveMotor[i].setInverted(false);			
			DriveMotor[i].setSensorPhase(false);
			DriveMotor[i].setNeutralMode(NeutralMode.Brake);
		}
		
		DriveMotor[DrvMap.RtMstr].setInverted(true);
		DriveMotor[DrvMap.RtSlv].setInverted(true);
		  
        DriveMotor[DrvMap.RtSlv].follow(DriveMotor[DrvMap.RtMstr]);
        DriveMotor[DrvMap.LtSlv].follow(DriveMotor[DrvMap.LtMstr]);

        for (int i = 0; i < DrvMap.NumOfMtrs; i++) {
			setDrvInvrtd(i, DriveMotor[i].getInverted());		
            captureDrvEncdrCntAll();
			VaDRV_Cnt_DrvEncdrZeroPstn[i] = VaDRV_Cnt_DrvEncdrPstn[i];			
        }

    }


    /*********************************************/
	/*     Subsystem Device Interfaces           */
	/*********************************************/

	public AHRS getNavX() {
		return mNavX;
	}

	public TalonFX getDriveMotor(int i) {
		return DriveMotor[i];
	}



    /*********************************************/
	/*     NAV/Gyro Interfaces                   */
	/*********************************************/
	
	public void captureGyroAngRaw() {
		VeDRV_Deg_NAV_SnsdAng = mNavX.getAngle();
	}

	public double getGyroAngRaw() {
		return(VeDRV_Deg_NAV_SnsdAng);
	}

    public double getGyroAngNorm() {
        double Le_Deg_Ang = VeDRV_Deg_NAV_SnsdAng;
        Le_Deg_Ang %= 360;
        if (Le_Deg_Ang < 0) Le_Deg_Ang = 360 + Le_Deg_Ang;
        return (Le_Deg_Ang);
    }

    public double getGyroAngErr() {
        double Le_Deg_Ang = (VeDRV_Deg_NAV_DsrdAng % 360) - (VeDRV_Deg_NAV_SnsdAng % 360);
        return(Le_Deg_Ang);
    }

	public double getGyroRate() {
		return(mNavX.getRate());
	}

	public double getNAV_DsrdAng() {
		return(VeDRV_Deg_NAV_DsrdAng);
	}

	public void resetGyro() {
		mNavX.reset();
	}

	public void setNAV_DsrdAng(double LeDRV_Deg_NAV_DsrdAng) {
		VeDRV_Deg_NAV_DsrdAng = LeDRV_Deg_NAV_DsrdAng;
	}

	public void zeroGyro() {
		setNAV_DsrdAng(getGyroAngRaw());
	}	



    /*********************************************/
	/* Public Subsystem Method Definitions       */
	/*********************************************/

	@Override
	public void periodic() {
	    // This method will be called once per scheduler run
	    /* Update SmartDashboard with Data */
	    // captureGyroAngRaw();
        // captureDrvEncdrCntAll();

	    if (K_DRV.KeDRV_b_DebugEnbl == true)  {
	        updateSmartDash();	
	    }
	
	}


    public void TankDrive(double Le_r_NormPwrRqstFwd, double Le_r_NormPwrRqstRot, double Le_r_NormPwrHdgCorr) {
		double Le_r_NormPwrCmndLt, Le_r_NormPwrCmndRt, Le_r_NormPwrMax;

		VeDRV_r_NormPwrRqstFwd = Le_r_NormPwrRqstFwd;
		VeDRV_r_NormPwrRqstRot = Le_r_NormPwrRqstRot;
		VeDRV_r_NormPwrHdgCorr = Le_r_NormPwrHdgCorr;

        Le_r_NormPwrCmndLt =  VeDRV_r_NormPwrRqstFwd + VeDRV_r_NormPwrRqstRot + VeDRV_r_NormPwrHdgCorr;
        Le_r_NormPwrCmndRt =  VeDRV_r_NormPwrRqstFwd - VeDRV_r_NormPwrRqstRot - VeDRV_r_NormPwrHdgCorr;

		/* Normalize Max Power to +/- 1.0 if either Commaned Power is > +/- 1.0 */
		Le_r_NormPwrMax = Math.max(Math.abs(Le_r_NormPwrCmndLt), Math.abs(Le_r_NormPwrCmndRt));
		if (Le_r_NormPwrMax > 1.0) {
		    Le_r_NormPwrCmndLt = Le_r_NormPwrCmndLt/Le_r_NormPwrMax;
		    Le_r_NormPwrCmndRt = Le_r_NormPwrCmndRt/Le_r_NormPwrMax;
		}

		/* Apply Max Rate Limiting to Prevent Overdriving the Motors/Drivers */
		// todo: Apply Max Rate Limiting
		Le_r_NormPwrCmndLt = rfsLIB.LimRateOnInc(Le_r_NormPwrCmndLt, VeDRV_r_NormPwrCmndLt, K_DRV.KeDRV_r_DrvNormPwrLimMaxDelt);
        VeDRV_r_NormPwrCmndLt = Le_r_NormPwrCmndLt;
		Le_r_NormPwrCmndRt = rfsLIB.LimRateOnInc(Le_r_NormPwrCmndRt, VeDRV_r_NormPwrCmndRt, K_DRV.KeDRV_r_DrvNormPwrLimMaxDelt);
        VeDRV_r_NormPwrCmndRt = Le_r_NormPwrCmndRt;

        cmndDrvMtr(VeDRV_r_NormPwrCmndLt, VeDRV_r_NormPwrCmndRt);
    }


    /**
     * Method: PID_DrvCtrl - Drive System: Control Drive Longitudinal Control
     * Drive with a PID controller using the Encoder Counts as a Reference.
     * @param Le_Cnt_CL_ErrPstn The error term (from the input device, encoder counts) for drive distance.
     */
    public double PID_DrvCtrl(double Le_Cnt_CL_PstnErr) {
		double Le_r_CL_CorrNormPwr;
		Le_r_CL_CorrNormPwr = VsDRV_PID_Rot.calculate(Le_Cnt_CL_PstnErr);
		Le_r_CL_CorrNormPwr = MathUtil.clamp(Le_r_CL_CorrNormPwr, VeDRV_r_PID_DrvPwrOutMin, VeDRV_r_PID_DrvPwrOutMax);
		return(Le_r_CL_CorrNormPwr);
	}


    /**
     * Method: PID_RotCtrl - Drive System: Control Drive Rotational Control
	 * with a PID controller using the Gyro as a Reference.
     * @param Le_Deg_CL_RotErr The error term (from the input device, generally gyroscope) for rotation
	 * and heading correction.
     */
    public double PID_RotCtrl(double Le_Deg_CL_RotErr) {
		double Le_r_CL_CorrNormPwr;
		Le_r_CL_CorrNormPwr = VsDRV_PID_Rot.calculate(Le_Deg_CL_RotErr);
		Le_r_CL_CorrNormPwr = MathUtil.clamp(Le_r_CL_CorrNormPwr, VeDRV_r_PID_RotPwrOutMin, VeDRV_r_PID_RotPwrOutMax);
		return(Le_r_CL_CorrNormPwr);
	  }


	public void configPID_DrvCtrl(double Le_Cnt_DrvEncdrTgt) {
      // Set up forward pid:
      setDrvSetpoint(Le_Cnt_DrvEncdrTgt);
      setDrvTolerance(5, 10);
	  setDrvOutputRange(-1, 1);
	  
	}


	public void configPID_RotCtrlHdgCorr(double Le_Deg_RotHdgTgt) {
		// Rotation PID (has continuous input)
		VsDRV_PID_Rot.setPID(K_DRV.KeDRV_k_CL_DrvPropGxRot, K_DRV.KeDRV_k_CL_DrvIntglGxRot, K_DRV.KeDRV_k_CL_DrvDerivGxRot);
		setRotWraparoundInputRange(0, 360);
		setRotSetpoint(Le_Deg_RotHdgTgt);
		setRotTolerance(5, 5);
		setRotOutputRange(-1, 1);
	}  


	public void configPID_RotCtrl(double Le_Deg_RotAngTgt) {
		// Rotation PID (has continuous input)
		VsDRV_PID_Rot.setPID(K_DRV.KeDRV_k_CL_RotPropGx, K_DRV.KeDRV_k_CL_RotIntglGx, K_DRV.KeDRV_k_CL_RotDerivGx);
		setRotWraparoundInputRange(0, 360);
		setRotSetpoint(Le_Deg_RotAngTgt);
		setRotTolerance(5, 5);
		setRotOutputRange(-1, 1);
	}  



    /*********************************************/
	/*  Private Subsystem Method Definitions     */
	/*********************************************/

	/**
     * Method: cmndDrvMtr - Swerve Drive System: Gets the direction of the Drive Motor Speed
     * direction.
     * @param Le_r_NormPwrCmndLt  (double:  Drive Motor Normalized Power Command -  Left-Side Motors)
     * @param Le_r_NormPwrCmndRt  (double:  Drive Motor Normalized Power Command - Right-Side Motors)
     */
    private void cmndDrvMtr(double Le_r_NormPwrCmndLt, double Le_r_NormPwrCmndRt) {
		DriveMotor[DrvMap.LtMstr].set(ControlMode.PercentOutput, Le_r_NormPwrCmndLt);
        DriveMotor[DrvMap.RtMstr].set(ControlMode.PercentOutput, Le_r_NormPwrCmndRt);
    }


  /**
    * Method: setDriveInverted - Swerve Drive System: Gets the direction of the Drive Motor Speed
    * direction.
    * @param Le_e_MtrIdx         (int:     Drive Motor Index)
    * @param Le_b_DrvMtrInvrtd   (boolean: Drive Motor Speed Inverted Indication)
    */
    private void setDrvInvrtd(int Le_e_MtrIdx, boolean Le_b_DrvMtrInvrtd) {
		VaDRV_b_MtrInvrtd[Le_e_MtrIdx] = Le_b_DrvMtrInvrtd;
	}
	
	private void captureDrvEncdrCnt(int Le_e_MtrIdx) {
		VaDRV_Cnt_DrvEncdrPstn[Le_e_MtrIdx] = (int)Math.round(DriveMotor[Le_e_MtrIdx].getSelectedSensorPosition());
	}  

	private void resetDrvEncdrPstn(int Le_e_MtrIdx) {
		DriveMotor[Le_e_MtrIdx].setSelectedSensorPosition(0);
	  }
  
	private void resetDrvZeroPstn(int Le_e_MtrIdx) {
		VaDRV_Cnt_DrvEncdrZeroPstn[Le_e_MtrIdx] = DriveMotor[Le_e_MtrIdx].getSelectedSensorPosition();
	}




	/*************************************************/
	/*  Public Set/Reset/Stop/Halt Interfaces        */
	/*************************************************/

    public void stopDrvMtrAll() {
        DriveMotor[DrvMap.LtMstr].set(ControlMode.PercentOutput, 0);
		DriveMotor[DrvMap.RtMstr].set(ControlMode.PercentOutput, 0);
    }

	public void captureDrvEncdrCntAll() {
		for (int i = 0; i < DrvMap.NumOfMtrs; i++ ) {
		    captureDrvEncdrCnt(i);
		}
	}

	public void resetDrvEncdrPstnAll() {
		for (int i = 0; i < DrvMap.NumOfMtrs; i++ ) {
			resetDrvEncdrPstn(i);
		}
	}

	public void resetDrvZeroPstnAll() {
		for (int i = 0; i < DrvMap.NumOfMtrs; i++ ) {
			resetDrvZeroPstn(i);
		}
	}

	public void resetDrvSysPID(){
		resetDrvPID();
		resetRotPID();
	}
  
	public void setDRV_r_PID_DrvPowCorr(double LeDRV_r_PID_DrvPowCorr) {
		VeDRV_r_PID_DrvPowCorr = LeDRV_r_PID_DrvPowCorr;
	}

	public void setDRV_r_PID_RotPowCorr(double LeDRV_r_PID_RotPowCorr) {
		VeDRV_r_PID_RotPowCorr = LeDRV_r_PID_RotPowCorr;
	}



	/*************************************************/
	/*  Public Get Interfaces                        */
	/*************************************************/

    /**
      * Method: getDrvInvrtd - Swerve Drive System: Gets the direction of the Drive Motor Speed
      * direction.
      * @param Le_e_MtrIdx          (int:     Drive Motor Index)
      * @return Le_b_DrvMtrInvrtd   (boolean: Drive Motor Speed Inverted Indication)
      */
    public boolean getDrvInvrtd(int Le_e_MtrIdx) {
		boolean Le_b_DrvMtrInvrtd = VaDRV_b_MtrInvrtd[Le_e_MtrIdx];
		return(Le_b_DrvMtrInvrtd);
	}

	public int getDrvEncdrCntsAbs(int Le_e_MtrIdx) {
		return (VaDRV_Cnt_DrvEncdrPstn[Le_e_MtrIdx]);
	}  
	
	public int getDrvEncdrCntsRel(int Le_e_MtrIdx) {
		return (VaDRV_Cnt_DrvEncdrPstn[Le_e_MtrIdx] - VaDRV_Cnt_DrvEncdrZeroPstn[Le_e_MtrIdx]);
	}
  
	public double getDrvInchesPerEncdrCnts(int Le_Cnts_DrvEncdrCnts) {
		double Le_l_DrvWhlDistInches;
	    Le_l_DrvWhlDistInches = (Le_Cnts_DrvEncdrCnts / K_DRV.KeDRV_Cnt_DrvEncdrCntsPerRev) * K_DRV.KeDRV_l_DrvWhlDistPerRev;
		return (Le_l_DrvWhlDistInches);
	}  
		
	public int getDrvEncdrCntsPerInches(double Le_l_DrvWhlDistInches) {
		double Le_Cnts_DrvEncdr;  
		Le_Cnts_DrvEncdr = (Le_l_DrvWhlDistInches / K_DRV.KeDRV_l_DrvWhlDistPerRev) * K_DRV.KeDRV_Cnt_DrvEncdrCntsPerRev;
		return ((int) Math.round(Le_Cnts_DrvEncdr));
	}
	
	public double getDrvDistTravelled(int Le_e_MtrIdx) { 
		int Le_Cnts_DrvEncdrCntDelt;  
		Le_Cnts_DrvEncdrCntDelt = DriveMotor[Le_e_MtrIdx].getSelectedSensorPosition() - VaDRV_Cnt_DrvEncdrZeroPstn[Le_e_MtrIdx];
		return (getDrvInchesPerEncdrCnts(Le_Cnts_DrvEncdrCntDelt));
	}
	
	public double getDrvDistance(int Le_e_MtrIdx) {
		int Le_Cnts_DrvEncdrCnt = DriveMotor[Le_e_MtrIdx].getSelectedSensorPosition();
		if (VaDRV_b_MtrInvrtd[Le_e_MtrIdx] == true)
		    Le_Cnts_DrvEncdrCnt = -Le_Cnts_DrvEncdrCnt;
		return getDrvInchesPerEncdrCnts(Le_Cnts_DrvEncdrCnt);
	}
	





    /*************************************************/
	/*  Drive DRV PID Methods for configuring PID        */
	/*************************************************/
	
	public void setDrvSetpoint(double setpoint){
		VsDRV_PID_Drv.setSetpoint(setpoint);
	}
	public void setDrvTolerance(double positionTolerance, double velocityTolerance){
		VsDRV_PID_Drv.setTolerance(positionTolerance, velocityTolerance);
	}
	public void setDrvWraparoundInputRange(double min, double max){
		VsDRV_PID_Drv.enableContinuousInput(min, max);
	}
	public void setDrvAccumulationRange(double min, double max){
		VsDRV_PID_Drv.setIntegratorRange(min, max);
	} 
	public void setDrvOutputRange(double minOutput, double maxOutput) {
		VeDRV_r_PID_DrvPwrOutMin = minOutput;
		VeDRV_r_PID_DrvPwrOutMax = maxOutput;
	}
	public double getDrvErrorDerivative(){
		return VsDRV_PID_Drv.getVelocityError();
	}
	public boolean getDrvAtSetpoint(){
		return VsDRV_PID_Drv.atSetpoint();
	}
	/**
	* Clear all I accumulation, disable continuous input, and set all 
	* setpoints to 0.
	*/
	public void resetDrvPID(){
	    // clear I accumulation
		VsDRV_PID_Drv.reset();
  
		// reset to noncontinuous input
		VsDRV_PID_Drv.disableContinuousInput();
  
		// set all setpoints to 0
		VsDRV_PID_Drv.setSetpoint(0);
  
		// set all I accumulation ranges to defaults
		VsDRV_PID_Drv.setIntegratorRange(-1.0, 1.0);
  
		VsDRV_PID_Drv.setTolerance(0.05, Double.POSITIVE_INFINITY);
  
		VeDRV_r_PID_DrvPwrOutMin = -1;
		VeDRV_r_PID_DrvPwrOutMax = 1;
	}


    /*************************************************/
	/*  Drive ROT PID Methods for configuring PID    */
	/*************************************************/
	
	public void setRotSetpoint(double setpoint){
		VsDRV_PID_Rot.setSetpoint(setpoint);
	}
	public void setRotTolerance(double positionTolerance, double velocityTolerance){
		VsDRV_PID_Rot.setTolerance(positionTolerance, velocityTolerance);
	}
	public void setRotWraparoundInputRange(double min, double max){
		VsDRV_PID_Rot.enableContinuousInput(min, max);
	}
	public void setRotAccumulationRange(double min, double max){
		VsDRV_PID_Rot.setIntegratorRange(min, max);
	} 
	public void setRotOutputRange(double minOutput, double maxOutput) {
		VeDRV_r_PID_RotPwrOutMin = minOutput;
		VeDRV_r_PID_RotPwrOutMax = maxOutput;
	}
	public double getRotErrorDerivative(){
		return VsDRV_PID_Rot.getVelocityError();
	}
	public boolean getRotAtSetpoint(){
		return VsDRV_PID_Rot.atSetpoint();
	}
	/**
	* Clear all I accumulation, disable continuous input, and set all 
	* setpoints to 0.
	*/
	public void resetRotPID(){
	    // clear I accumulation
		VsDRV_PID_Rot.reset();
  
		// reset to noncontinuous input
		VsDRV_PID_Rot.disableContinuousInput();
  
		// set all setpoints to 0
		VsDRV_PID_Rot.setSetpoint(0);
  
		// set all I accumulation ranges to defaults
		VsDRV_PID_Rot.setIntegratorRange(-1.0, 1.0);
  
		VsDRV_PID_Rot.setTolerance(0.05, Double.POSITIVE_INFINITY);
  
		VeDRV_r_PID_RotPwrOutMin = -1;
		VeDRV_r_PID_RotPwrOutMax = 1;
	}



    /*************************************************/
	/*     Subsystem Instrumenation Display          */
	/*************************************************/

	private void updateSmartDash() {
		/* Print to SmartDashboard */
	
		SmartDashboard.putNumber("Gyro Snsd Ang " ,  VeDRV_Deg_NAV_SnsdAng);
		SmartDashboard.putNumber("Gyro Dsrd Ang " ,  VeDRV_Deg_NAV_DsrdAng);

		SmartDashboard.putNumber("Pwr Rqst Fwd " ,   VeDRV_r_NormPwrRqstFwd);
		SmartDashboard.putNumber("Pwr Rqst Rot " ,   VeDRV_r_NormPwrRqstRot);
		SmartDashboard.putNumber("Pwr Rqst Hdg " ,   VeDRV_r_NormPwrHdgCorr);
		SmartDashboard.putNumber("Pwr Cmnd Lt " ,    VeDRV_r_NormPwrCmndLt);
		SmartDashboard.putNumber("Pwr Cmnd Rt " ,    VeDRV_r_NormPwrCmndRt);

		SmartDashboard.putNumber("Pwr PID Drv " ,    VeDRV_r_PID_DrvPowCorr);
		SmartDashboard.putNumber("PID SetPt Drv " ,  VeDRV_Cnt_PID_DrvPstnCmnd);
		SmartDashboard.putNumber("Pwr PID Rot " ,    VeDRV_r_PID_RotPowCorr);
		SmartDashboard.putNumber("PID SetPt Rot " ,  VeDRV_Deg_PID_RotAngCmnd);


		for (int i = 0; i < DrvMap.NumOfMtrs; i++)  {	
		   SmartDashboard.putNumber("Drv Encdr Cnts " + i ,      VaDRV_Cnt_DrvEncdrPstn[i]);	   
		   SmartDashboard.putNumber("Drv Encdr Zero Pstn " + i , VaDRV_Cnt_DrvEncdrZeroPstn[i]);
		   SmartDashboard.putNumber("Drvn Encdr Dist " + i ,     getDrvDistance(i));

//		   SmartDashboard.putBoolean("Drv Mtr Dir Trig " + i , SwrvDrvMod[i].getDrvMtrDirctnTrig());
		}
        
	}



} 
