package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.InputMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.calibrations.K_SWRV;
 

public class SwerveDriveModule extends SubsystemBase {

  public enum TeMtrDirctn {
        Fwd,
        Rwd;
   }

	private final int Me_i_ModIdx;

    private final CANSparkMax Ms_h_RotMtr;
    private final CANEncoder Ms_h_RotEncdr;

    private final TalonFX Ms_h_DrvMtr;

    private final double Me_Deg_RotEncdrZeroOfst;
    private double Me_t_RotMtrStlInitTm;
 
    private double Me_r_DrvEncdrZeroPstn;
    private TeMtrDirctn Me_e_DrvMtrDirctn;
    private boolean Me_b_DrvMtrDirctnLtch;
    private boolean Me_b_DrvMtrDirctnUpdTrig;


	SwerveDriveModule(int Le_i_ModIdx, CANSparkMax Ls_h_RotMtr, TalonFX Ls_h_DrvMtr, double Le_Deg_RotZeroOfst) {
        Me_i_ModIdx = Le_i_ModIdx;
        
		Ms_h_RotMtr = Ls_h_RotMtr;
        Ms_h_RotEncdr = Ms_h_RotMtr.getEncoder();

        Me_Deg_RotEncdrZeroOfst = Le_Deg_RotZeroOfst;
        Me_t_RotMtrStlInitTm = 0;

        Ms_h_DrvMtr = Ls_h_DrvMtr;
        
        Me_r_DrvEncdrZeroPstn = (double)Ms_h_DrvMtr.getSelectedSensorPosition();
        Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
        Me_b_DrvMtrDirctnLtch = false;
        Me_b_DrvMtrDirctnUpdTrig = false;


        /*****************************************************************/
        /* Rotation Control PID Controller Configuration                 */
        /*****************************************************************/
        CANPIDController Ms_h_ROT_PID_Cntrlr = Ls_h_RotMtr.getPIDController();
        Ms_h_RotMtr.restoreFactoryDefaults();

		// set PID coefficients
		Ms_h_ROT_PID_Cntrlr.setP(K_SWRV.KeSWRV_K_RotProp);
		Ms_h_ROT_PID_Cntrlr.setI(K_SWRV.KeSWRV_K_RotIntgl);
		Ms_h_ROT_PID_Cntrlr.setD(K_SWRV.KeSWRV_K_RotDeriv);
		Ms_h_ROT_PID_Cntrlr.setIZone(K_SWRV.KeSWRV_e_RotIntglErrMaxEnbl);
		Ms_h_ROT_PID_Cntrlr.setFF(K_SWRV.KeSWRV_K_RotFdFwd);
		Ms_h_ROT_PID_Cntrlr.setOutputRange(K_SWRV.KeSWRV_r_RotNormOutMin, K_SWRV.KeSWRV_r_RotNormOutMax);
           
        // Set Idle Mode
        Ls_h_RotMtr.setIdleMode(IdleMode.kBrake);

        // Set Sensor Type
//      Ms_h_ROT_PID_Cntrlr.setParameter(ConfigParameter.kSensorType, SensorType.kHallSensor.value);


        // Set amperage limits
        Ls_h_RotMtr.setSmartCurrentLimit(K_SWRV.KeSWRV_I_RotDrvrLmtMaxPri);
        Ls_h_RotMtr.setSecondaryCurrentLimit(K_SWRV.KeSWRV_I_RotDrvrLmtMaxSec,0);
        Ls_h_RotMtr.setCANTimeout(K_SWRV.KeSWRV_t_RotCAN_TmeOut);


        
        /*****************************************************************/
        /* Drive Control PID Controller Configurations                   */
        /*****************************************************************/
        Ls_h_DrvMtr.configFactoryDefault();
        Ls_h_DrvMtr.setInverted(false);
        Ls_h_DrvMtr.setSensorPhase(false);

		// set PID coefficients
		Ls_h_DrvMtr.config_kP(0, K_SWRV.KeSWRV_K_DrvProp);
		Ls_h_DrvMtr.config_kI(0, K_SWRV.KeSWRV_K_DrvIntgl);
		Ls_h_DrvMtr.config_kD(0, K_SWRV.KeSWRV_K_DrvDeriv);
		Ls_h_DrvMtr.config_IntegralZone(0, K_SWRV.KeSWRV_e_DrvIntglErrMaxEnbl);
		Ls_h_DrvMtr.config_kF(0, K_SWRV.KeSWRV_K_DrvFdFwd);
        Ls_h_DrvMtr.configMotionCruiseVelocity(K_SWRV.KeSWRV_n_Drv_MM_CruiseVel);
        Ls_h_DrvMtr.configMotionAcceleration(K_SWRV.KeSWRV_a_Drv_MM_MaxAccel);
//      Ls_h_DrvMtr.configIntegratedSensorAbsoluteRange([K_SWRV.KeSWRV_r_DrvNormOutMin, K_SWRV.KeSWRV_r_DrvNormOutMax), (int)0);

        // Set Idle Mode
        Ls_h_DrvMtr.setNeutralMode(NeutralMode.Brake);

        // Set Sensor Type
        Ls_h_DrvMtr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
 
        // Set amperage limits
        Ls_h_DrvMtr.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxPri, 15, 0.5));
        Ls_h_DrvMtr.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxSec, 25 ,1.0));
//      Ls_h_DrvMtr.setCANTimeout(K_SWRV.KeSWRV_t_DrvCAN_TmeOut);

    }
	

    public CANSparkMax getRotMtr() {
        return Ms_h_RotMtr;
    }

    public TalonFX getDrvMtr() {
        return Ms_h_DrvMtr;
    }

    public void disableRobotInitMd() {
        Me_t_RotMtrStlInitTm = Long.MAX_VALUE;
    }

   /** Method: getRotActAng - Swerve Drive System - Gets the Rotational
      * Motor Actual Feedback Angle that has Zero Offset and Drive
      * Motor Correction applied.  
      * @return Le_r_RotEncdrActPstn (double: caddy rotation motor encoder nominal position)	
      */	  
      public double getRotEncdrActPstn() {
        double Le_r_RotEncdrActPstn = Ms_h_RotEncdr.getPosition();
        return Le_r_RotEncdrActPstn;
    }


    /** Method: getRotActAng - Swerve Drive System - Gets the Rotational
      * Motor Actual Feedback Angle that has Zero Offset and Drive
      * Motor Correction applied.  
      * @return Le_Deg_RotAngAct (double: caddy rotation actual angle)	
      */	  
    public double getRotActAng() {
        double Le_Deg_RotAngActRaw, Le_Deg_RotAngActCorr, Le_Deg_RotAngAct; 

        Le_Deg_RotAngActRaw = Ms_h_RotEncdr.getPosition() * (360.0 / K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat);

        Le_Deg_RotAngActCorr = (Le_Deg_RotAngActRaw % 360) - Me_Deg_RotEncdrZeroOfst;
        if (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd)
            Le_Deg_RotAngActCorr = (Le_Deg_RotAngActCorr + 180);
        Le_Deg_RotAngActCorr = Le_Deg_RotAngActCorr % 360;

        if (Le_Deg_RotAngActCorr < 0) {
            Le_Deg_RotAngAct = 360 -  Le_Deg_RotAngActCorr;
        }
        else {
            Le_Deg_RotAngAct = Le_Deg_RotAngActCorr;
        } 

        return Le_Deg_RotAngAct;
    }


    /** Method: setRotTgtAng - Swerve Drive System sets the Rotational
      * Motor Target Angle.  Converts the Target Angle from Degrees
      * of Caddy Position to Normalized Caddy Position.  
      * @param Le_Deg_RotAngTgt (double: desired caddy target angle)	
      */	  
    public void setRotTgtAng(double Le_Deg_RotAngTgt) {
        double Le_r_RotAngTgt, Le_r_RotAngTgtMtr;
        Le_r_RotAngTgt = (Le_Deg_RotAngTgt/360);
        Le_r_RotAngTgtMtr = Le_r_RotAngTgt * K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat;

        Ms_h_RotMtr.getPIDController().setReference(Le_r_RotAngTgtMtr, ControlType.kPosition);

        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            /* Print to SmartDashboard */
            SmartDashboard.putNumber("Module Target Angle Caddy (Degrees) " + Me_i_ModIdx, Le_Deg_RotAngTgt);
            SmartDashboard.putNumber("Module Target Angle Caddy (Norm Rot) " + Me_i_ModIdx, Le_r_RotAngTgt);
            SmartDashboard.putNumber("Module Target Angle Motor (Norm Rot) " + Me_i_ModIdx, Le_r_RotAngTgtMtr);
        }

    }


     /** Method: invertDrvMtrDirctn - Swerve Drive System - Inverts the
      * Indicator of the Direction of the Drive Motor. 
      */   
    private void invertDrvMtrDirctn()  {
        if (Me_e_DrvMtrDirctn == TeMtrDirctn.Fwd) {
          Me_e_DrvMtrDirctn = TeMtrDirctn.Rwd;
        }
        else /* (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd) */ {
          Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
        }
     }

 
     /** Method: dtrmnDrvMtrDirctn - Swerve Drive System determination of
      * the Drive Motor Direction to obtain the Desired Swerbe Drive Caddy
      * Rotation Angle in the most efficient manner. 
      * @param Le_Deg_RotAngTgtRaw (double: desired caddy target angle)
      * @param Le_b_SwrvRotRqstActv (boolean: swerve caddy rotation request active)	
      */   
    public void dtrmnDrvMtrDirctn(double Le_Deg_RotAngTgtRaw, boolean Le_b_SwrvRotRqstActv) {
        double Le_Deg_RotAngTgt;
        double Le_r_RotAngTgt;
        double Le_Deg_RotAngActRaw;
        double Le_Deg_RotAngAct;
        double Le_Deg_RotErrAbs;      

        // Calculate remainder after dividing by 360 degrees
        Le_Deg_RotAngTgt = Le_Deg_RotAngTgtRaw % 360; 
        
        // Get Current Position, then math it to be out of 360 degrees
        Le_Deg_RotAngAct = getRotActAng();

        Le_Deg_RotErrAbs = Math.abs(Le_Deg_RotAngTgt - Le_Deg_RotAngAct);


        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            /* Print to SmartDashboard */
          SmartDashboard.putNumber("Module Actual Angle (Enc Counts) " + Me_i_ModIdx, Ms_h_RotEncdr.getPosition());
          SmartDashboard.putNumber("Module Actual Angle (Degrees)    " + Me_i_ModIdx, Le_Deg_RotAngAct);
          SmartDashboard.putNumber("Module Angle Error (Degrees)     " + Me_i_ModIdx, Le_Deg_RotErrAbs);
          SmartDashboard.putString("Module Driver Motor Init Direction  " + Me_i_ModIdx, Me_e_DrvMtrDirctn.toString());
        }


        if ((K_SWRV.KeSWRV_b_DrvMtrRotDirctnInvertInhb == true) && (Le_b_SwrvRotRqstActv == true)) {
            Me_b_DrvMtrDirctnUpdTrig = false;
        }
        else if ((Le_Deg_RotErrAbs > 90) && (Le_Deg_RotErrAbs < 270)) {
            Me_b_DrvMtrDirctnUpdTrig = true;
            invertDrvMtrDirctn();
        }
        else {
            Me_b_DrvMtrDirctnUpdTrig = false;
        }


        if (K_SWRV.KeSWRV_b_DebugEnbl == true)  {
            /* Print to SmartDashboard */
          SmartDashboard.putBoolean("Module Direction Update Trigger  " + Me_i_ModIdx, Me_b_DrvMtrDirctnUpdTrig);
          SmartDashboard.putString("Module Driver Motor Update  Direction    " + Me_i_ModIdx, Me_e_DrvMtrDirctn.toString());
        }
        
    }


    public void setRotEncdrTgtPstn(double Le_r_EncdrPstn) {
        Ms_h_RotMtr.getPIDController().setReference(Le_r_EncdrPstn, ControlType.kPosition);
    }

    public void setDrvMtrSpd(double Le_r_MtrSpd) {
        double Le_r_DirctnSclr = (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd ? -1 : 1); 

        Ms_h_DrvMtr.set(ControlMode.Velocity, (Le_r_DirctnSclr * Le_r_MtrSpd));
    }

    public void resetDrvZeroPstn() {
        Me_r_DrvEncdrZeroPstn = (double)Ms_h_DrvMtr.getSelectedSensorPosition();
    }
 
    public void resetRotEncdr() {
        Ms_h_RotEncdr.setPosition(0);
    }

    public double getDrvEncdrDelt() {
        return ((double)Ms_h_DrvMtr.getSelectedSensorPosition() - Me_r_DrvEncdrZeroPstn);
    }

    public double getDrvInchesPerEncdrCnts(double Le_r_DrvEncdrNormCnts) {
         return Le_r_DrvEncdrNormCnts / K_SWRV.KeSWRV_Cf_DrvMtrEncdrCntsToInch;
    }

    public int getDrvEncdrCntsPerInches(double Le_l_DrvWhlDistInches) {
         return (int) Math.round(Le_l_DrvWhlDistInches * K_SWRV.KeSWRV_Cf_DrvMtrEncdrCntsToInch);
    }

    public double getDrvDist() { 
        double Le_r_DrvEncdrNormCnts = (double)Ms_h_DrvMtr.getSelectedSensorPosition();
        return getDrvInchesPerEncdrCnts(Le_r_DrvEncdrNormCnts);
    }
}
