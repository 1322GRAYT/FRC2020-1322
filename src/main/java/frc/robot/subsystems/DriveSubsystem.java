/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.SwrvMap;
import frc.robot.calibrations.K_DRV;

/**
 * Add your docs here.
 */
public class DriveSubsystem implements Subsystem {

    private TalonFX[] DriveMotor = new TalonFX[] { 
		new TalonFX(Constants.SWRV_FR_RT_DRV),
		new TalonFX(Constants.SWRV_FR_LT_DRV),
		new TalonFX(Constants.SWRV_RR_LT_DRV),
		new TalonFX(Constants.SWRV_RR_RT_DRV)
	};

    private double VeDRV_r_NormPwrRqstFwd;
    private double VeDRV_r_NormPwrRqstRot;

    private double VeDRV_r_NormPwrCmndLt;
    private double VeDRV_r_NormPwrCmndRt;

    private PIDController VsDRV_PID_Rot;
    private double VeDRV_r_PID_RotPwrOutMin;
    private double VeDRV_r_PID_RotPwrOutMax;



	/*******************************/
	/* Subsystem Constructor       */
    /*******************************/
    public DriveSubsystem() {

        VeDRV_r_NormPwrRqstFwd = 0;
        VeDRV_r_NormPwrRqstRot = 0;    

        VeDRV_r_NormPwrCmndLt  = 0;
        VeDRV_r_NormPwrCmndRt  = 0;    

		VsDRV_PID_Rot  = new PIDController(K_DRV.KeDRV_k_CL_PropGx_Rot,  K_DRV.KeDRV_k_CL_IntglGx_Rot,  K_DRV.KeDRV_k_CL_DerivGx_Rot);
		VeDRV_r_PID_RotPwrOutMin = -1;
		VeDRV_r_PID_RotPwrOutMax =  1;


        DriveMotor[SwrvMap.LtRr].follow(DriveMotor[SwrvMap.LtFt]);
        DriveMotor[SwrvMap.RtRr].follow(DriveMotor[SwrvMap.RtFt]);

        /*****************************************************************/
        /* Drive Control PID Controller Configurations */
        /*****************************************************************/
        for (int i = 0; i < SwrvMap.NumOfCaddies; i++) {
            DriveMotor[i].configFactoryDefault();
            DriveMotor[i].setInverted(false);
            DriveMotor[i].setSensorPhase(false);
        }


    }




    /*********************************************/
	/*     Subsystem Method Definitions          */
	/*********************************************/

	@Override
	public void periodic() {
	  // This method will be called once per scheduler run
	  /* Update SmartDashboard with Data */

	  if (K_DRV.KeDRV_b_DebugEnbl == true)  {
	    updateSmartDash();	
	  }
	
	}



    public void TankDrive(double Le_r_NormPwrRqstFwd, double Le_r_NormPwrRqstRot) {
        double Le_r_NormPwrCmndLt, Le_r_NormPwrCmndRt;

        VeDRV_r_NormPwrRqstFwd =  applyDB_NormPwr(Le_r_NormPwrRqstFwd, K_DRV.KeDRV_r_DB_CntlrThrshRot);
        VeDRV_r_NormPwrRqstRot =  applyDB_NormPwr(Le_r_NormPwrRqstRot, K_DRV.KeDRV_r_DB_CntlrThrshRot);

        if (Math.abs(VeDRV_r_NormPwrRqstFwd) >= K_DRV.KeDRV_r_DrvRqstOvrrdFwd) {
            Le_r_NormPwrCmndLt =  VeDRV_r_NormPwrRqstFwd;
            Le_r_NormPwrCmndRt =  VeDRV_r_NormPwrRqstFwd;
        }
        else if (Math.abs(VeDRV_r_NormPwrRqstRot) >= K_DRV.KeDRV_r_DrvRqstOvrrdRot) {
            Le_r_NormPwrCmndLt =  VeDRV_r_NormPwrRqstRot;
            Le_r_NormPwrCmndRt = -VeDRV_r_NormPwrRqstRot;
        }
        else {
            Le_r_NormPwrCmndLt =  VeDRV_r_NormPwrRqstFwd + VeDRV_r_NormPwrRqstRot;
            Le_r_NormPwrCmndRt =  VeDRV_r_NormPwrRqstFwd - VeDRV_r_NormPwrRqstRot;
        }

        CommandDriveMotor(Le_r_NormPwrCmndLt, Le_r_NormPwrCmndRt);
    }


    public void CommandDriveMotor(double Le_r_NormPwrCmndLt, double Le_r_NormPwrCmndRt) {
        VeDRV_r_NormPwrCmndLt = Le_r_NormPwrCmndLt;
        VeDRV_r_NormPwrCmndRt = Le_r_NormPwrCmndRt;

        DriveMotor[SwrvMap.LtFt].set(ControlMode.PercentOutput, VeDRV_r_NormPwrCmndLt);
        DriveMotor[SwrvMap.RtFt].set(ControlMode.PercentOutput, VeDRV_r_NormPwrCmndRt);

    }


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


    public void stopDrvMtrs() {
        DriveMotor[SwrvMap.LtFt].set(ControlMode.PercentOutput, 0);
        DriveMotor[SwrvMap.RtFt].set(ControlMode.PercentOutput, 0);
    }






    /*************************************************/
	/*  Drive PID Methods for configuring PID        */
	/*************************************************/
	// All the PID System methods. 
	
	public void setRotationSetpoint(double setpoint){
		VsDRV_PID_Rot.setSetpoint(setpoint);
	}
	public void setRotationTolerance(double positionTolerance, double velocityTolerance){
		VsDRV_PID_Rot.setTolerance(positionTolerance, velocityTolerance);
	}
	public void setRotationWraparoundInputRange(double min, double max){
		VsDRV_PID_Rot.enableContinuousInput(min, max);
	}
	public void setRotationAccumulationRange(double min, double max){
		VsDRV_PID_Rot.setIntegratorRange(min, max);
	} 
	public void setRotationOutputRange(double minOutput, double maxOutput) {
		VeDRV_r_PID_RotPwrOutMin = minOutput;
		VeDRV_r_PID_RotPwrOutMax = maxOutput;
	}
	public double getRotationErrorDerivative(){
		return VsDRV_PID_Rot.getVelocityError();
	}
	public boolean rotationAtSetpoint(){
		return VsDRV_PID_Rot.atSetpoint();
	}
	/**
	* Clear all I accumulation, disable continuous input, and set all 
	* setpoints to 0.
	*/
	public void resetPID(){
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
/*	
		SmartDashboard.putNumber("Cntrlr Pwr Long " , VeSDRV_r_PwrLong);
		SmartDashboard.putNumber("Cntrlr Pwr Lat " ,  VeSDRV_r_PwrLat);
		SmartDashboard.putNumber("Cntrlr Pwr Rot " ,  VeSDRV_r_PwrRot);
*/	

/*
		for (int i = 0; i < SwrvMap.NumOfCaddies; i++)  {
	
		   SmartDashboard.putNumber("Drv Encdr Cnts " + i ,    SwrvDrvMod[i].getDrvEncdrCntsRel());	   
		   SmartDashboard.putString("Drv Mtr Dir " + i ,       SwrvDrvMod[i].getDrvMtrDirctn().toString());
		   SmartDashboard.putBoolean("Drv Mtr Dir Trig " + i , SwrvDrvMod[i].getDrvMtrDirctnTrig());
		   
		}
*/
        
	}



} 
