package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.InputMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.calibrations.K_SWRV;
import frc.robot.Constants.DriveShiftPos;



public class SwerveDriveModule extends SubsystemBase {

  public enum TeMtrDirctn {
      Fwd,
      Rwd;
   }

  public enum TeRotDirctn {
      CW,
      CCW;
  }

  public enum Te_RZL_St {
      Inactv,
      Init,
      SwpCW,
      SwpCCW,
      ZeroDtct,
      ZeroCptr,
      Cmplt;
  }  

  private final int Me_i_ModIdx;

  private final CANSparkMax  Ms_h_RotMtr;
  private final CANEncoder   Ms_h_RotEncdr;
  private final DigitalInput Ms_b_RotZeroDtcd;
   
  private final TalonFX Ms_h_DrvMtr;

  
  /* Rotation Motor Data */
  private int Me_r_RotModRevs;
  private double Me_Deg_RotAngActRaw;
  private double Me_r_RotEncdrZeroOfst;
  private double Me_t_RotMtrStlInitTm;
 
  /* Rotation Zero Position Learn */
  private Te_RZL_St Me_e_RZL_St;
  private double    Me_Deg_RZL_AngInit;
  private double    Me_Deg_RZL_AngSwp;
  private double    Me_Deg_RZL_AngTgt;
  private double    Me_r_RZL_Pwr;
  private Timer     Me_t_RZL_DtctTmrIntrsv  = new Timer();
  private Timer     Me_t_RZL_DtctTmrPassive = new Timer();
  private int       Me_Cnt_RZL_PassiveUpd;

  /* Drive Motor Data */
  private double Me_Cnt_DrvEncdrZeroPstn;
  private boolean Me_b_DrvMtrInverted;
  private TeMtrDirctn Me_e_DrvMtrDirctn;
  private boolean Me_b_DrvMtrDirctnUpdInhb;
  private boolean Me_b_DrvMtrDirctnUpdTrig;


  /*******************************/
  /* Subsystem Constructor */
  /*******************************/
  SwerveDriveModule(final int Le_i_ModIdx, final CANSparkMax Ls_h_RotMtr, final TalonFX Ls_h_DrvMtr, final DigitalInput Ls_b_RotZeroDtcd,final double Le_r_RotEncdrZeroOfst) {
    Me_i_ModIdx = Le_i_ModIdx;
  
    Ms_h_RotMtr = Ls_h_RotMtr;
    Ms_h_RotEncdr = Ms_h_RotMtr.getEncoder();
    Ms_b_RotZeroDtcd = Ls_b_RotZeroDtcd;

    Ms_h_DrvMtr = Ls_h_DrvMtr;

    Me_r_RotModRevs  = (int)0;    
    Me_Deg_RotAngActRaw   = 0;
    Me_r_RotEncdrZeroOfst = Le_r_RotEncdrZeroOfst;
    Me_t_RotMtrStlInitTm  = 0;

    Me_e_RZL_St = Te_RZL_St.Init;
    Me_Deg_RZL_AngInit = 0;
    Me_Deg_RZL_AngSwp = 0;
    Me_Deg_RZL_AngTgt = 0;
    Me_r_RZL_Pwr = 0;
    Me_t_RZL_DtctTmrIntrsv.reset();
    Me_t_RZL_DtctTmrPassive.reset();
    Me_Cnt_RZL_PassiveUpd = 0;

    
    Me_Cnt_DrvEncdrZeroPstn = Ms_h_DrvMtr.getSelectedSensorPosition();
    Me_b_DrvMtrInverted = false;
    Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
    Me_b_DrvMtrDirctnUpdInhb = false;
    Me_b_DrvMtrDirctnUpdTrig = false;

    /*****************************************************************/
    /* Rotation Control PID Controller Configuration */
    /*****************************************************************/
    final CANPIDController Ms_h_ROT_PID_Cntrlr = Ls_h_RotMtr.getPIDController();
    Ms_h_RotMtr.restoreFactoryDefaults();
    Ms_h_RotMtr.setInverted(false);

    // set PID coefficients
    Ms_h_ROT_PID_Cntrlr.setP(K_SWRV.KeSWRV_K_RotProp);
    Ms_h_ROT_PID_Cntrlr.setI(K_SWRV.KeSWRV_K_RotIntgl);
    Ms_h_ROT_PID_Cntrlr.setD(K_SWRV.KeSWRV_K_RotDeriv);
    Ms_h_ROT_PID_Cntrlr.setIZone(K_SWRV.KeSWRV_r_RotIntglErrMaxEnbl);
    Ms_h_ROT_PID_Cntrlr.setFF(K_SWRV.KeSWRV_K_RotFdFwd);
    Ms_h_ROT_PID_Cntrlr.setOutputRange(K_SWRV.KeSWRV_r_RotNormOutMin, K_SWRV.KeSWRV_r_RotNormOutMax);


    // Set Idle Mode
    Ms_h_RotMtr.setIdleMode(IdleMode.kBrake);

    // Set amperage limits
    Ms_h_RotMtr.setSmartCurrentLimit(K_SWRV.KeSWRV_I_RotDrvrLmtMaxPri);
    Ms_h_RotMtr.setSecondaryCurrentLimit(K_SWRV.KeSWRV_I_RotDrvrLmtMaxSec, 0);
    Ms_h_RotMtr.setCANTimeout(K_SWRV.KeSWRV_t_RotCAN_TmeOut);


    /*****************************************************************/
    /* Drive Control PID Controller Configurations */
    /*****************************************************************/
    Ms_h_DrvMtr.configFactoryDefault();
    Ms_h_DrvMtr.setInverted(false);
//  Ms_h_DrvMtr.setInverted(TalonFXInvertType.CounterClockwise);
    Ms_h_DrvMtr.setSensorPhase(false);

    // set PID coefficients
    Ms_h_DrvMtr.config_kP(0, K_SWRV.KeSWRV_K_DrvProp);
    Ms_h_DrvMtr.config_kI(0, K_SWRV.KeSWRV_K_DrvIntgl);
    Ms_h_DrvMtr.config_kD(0, K_SWRV.KeSWRV_K_DrvDeriv);
    Ms_h_DrvMtr.config_IntegralZone(0, K_SWRV.KeSWRV_n_DrvIntglErrMaxEnbl);
    Ms_h_DrvMtr.config_kF(0, K_SWRV.KeSWRV_K_DrvFdFwd);
    Ms_h_DrvMtr.configMotionCruiseVelocity(K_SWRV.KeSWRV_n_Drv_MM_CruiseVel);
    Ms_h_DrvMtr.configMotionAcceleration(K_SWRV.KeSWRV_a_Drv_MM_MaxAccel);
    // Ms_h_DrvMtr.configIntegratedSensorAbsoluteRange([K_SWRV.KeSWRV_r_DrvNormOutMin,
    // K_SWRV.KeSWRV_r_DrvNormOutMax), (int)0);

    // Set Idle Mode
    Ms_h_DrvMtr.setNeutralMode(NeutralMode.Brake);

    // Set Sensor Type
    Ms_h_DrvMtr.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);

    // Set amperage limits
    Ms_h_DrvMtr.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxPri, 30, 0.5));
    Ms_h_DrvMtr.configStatorCurrentLimit(new StatorCurrentLimitConfiguration(true, K_SWRV.KeSWRV_I_DrvDrvrLmtMaxSec, 30, 1.0));
    // Ms_h_DrvMtr.setCANTimeout(K_SWRV.KeSWRV_t_DrvCAN_TmeOut);

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


  

  /********************************************************/
  /* Rotation Control Motor/Encoder Interfaces            */
  /********************************************************/

  public double getRotMtrCurr() {
    return (Ms_h_RotMtr.getOutputCurrent());
  }
  
  /**
   * Method: getRotEncdrActPstn - Swerve Drive System - Gets the Rotational Motor
   * Actual Feedback Angle that has Zero Offset and Drive Motor Correction
   * applied. 
   * @return Le_r_RotEncdrActPstn (double: caddy rotation motor encoder nominal position)
   */
  public double getRotEncdrActPstn() {
    double Le_r_RotEncdrActPstn = Ms_h_RotEncdr.getPosition();
    return Le_r_RotEncdrActPstn;
  }


  /**
   * Method: correctRotEncdrActPstn - Swerve Drive System - Gets the Rotational Motor
   * Actual Encoder Position that has been corrected for the Rotation Control
   * Zero Position Encoder Offset.
   * @param  Le_r_RotEncdrActPstnRaw  (double: caddy rotation encoder position raw)
   * @return Le_r_RotEncdrActPstnCorr (double: caddy rotation encoder position zero corrected)
   */
  public double correctRotEncdrActPstn(double Le_r_RotEncdrActPstnRaw) {
    double Le_r_RotEncdrActPstnCorr;
    Le_r_RotEncdrActPstnCorr = Le_r_RotEncdrActPstnRaw - Me_r_RotEncdrZeroOfst;
    return (Le_r_RotEncdrActPstnCorr);
  }


  /**
   * Method: calcRotActAng - Swerve Drive System - Calculates the Rotational Motor
   * Actual Feedback Angle that has not been normalized for a single caddy
   * rotation. lso Stores away the number of Revs of wind-up if the Caddy had
   * wound up multiple times.
   * @param  Le_r_RotActEndrPstn  (double: caddy actual encoder position in revs)
   * @return Le_Deg_RotAngAct (double: caddy rotation actual angle raw - can have multiple rotations)
   */
  public double calcRotActAng(double Le_r_RotActEndrPstn) {
    Me_Deg_RotAngActRaw = (Le_r_RotActEndrPstn / K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat) * 360.0;
    Me_r_RotModRevs = (int)(Me_Deg_RotAngActRaw / 360);
    return (Me_Deg_RotAngActRaw);
  }


  /**
   * Method: normRotActAng - Swerve Drive System - Normailizes the
   * Rotation Actual Angle to a Single Caddy Rotation.
   * @param  Le_Deg_RotAngAct     (double: caddy rotation actual angle ) 
   * @return Le_Deg_RotAngActNorm (double: caddy rotation actual angle normalized)
   */
  public double normRotActAng(double Le_Deg_RotAngAct) {
    double Le_Deg_RotAngActNorm;
    Le_Deg_RotAngActNorm = Le_Deg_RotAngAct % 360;
    return (Le_Deg_RotAngActNorm);
  }

  /**
   * Method: cnvrtRotActAng - Swerve Drive System - Converts the
   * Rotation Actual Angle to the same Frame of Reference as the
   * Target Angle by shifting the Angle by +180 degrees, results
   * in an angles of [0 to 360] representing an actual [-180 to 180].
   * @param  Le_Deg_RotAngAct       (double: caddy rotation actual angle ) 
   * @return Le_Deg_RotAngActAdjstd (double: caddy rotation actual angle shifted by +180)
   */
  public double cnvrtRotActAng(double Le_Deg_RotAngAct) {
    double Le_Deg_RotAngActAdjstd;
    Le_Deg_RotAngActAdjstd = (Le_Deg_RotAngAct + 180) % 360;
    return (Le_Deg_RotAngActAdjstd);
  }


  /**
   * Method: cnvrtRotTgtAng - Swerve Drive System - Calculates the
   * Rotation Angle Target from the Calculated Angle by removing the 180
   * degree frame of reference shift that was applied to the Target Raw
   * (-180 to 180) up front.
   * @param  Le_Deg_RotAngCalc (double: caddy rotation calculated angle desired: 0 to 360 ) 
   * @return Le_Deg_RotAngTgt  (double: caddy rotation target angle requested: -180 to 180)
   */
  public double cnvrtRotTgtAng(double Le_Deg_RotAngCalc) {
    double Le_Deg_RotAngTgt;
    Le_Deg_RotAngTgt = (Le_Deg_RotAngCalc - 180);  // Convert from [0 to 360] back to [-180 to 180]
    return (Le_Deg_RotAngTgt);
  }

  
  /**
   * Method: calcRotTgtAngAbs - Swerve Drive System - Calculates the
   * Rotation Angle Target from the Calculated Angle taking into
   * account removing the 180 degree frame of reference shift that
   * was applied to the Target Raw (-180 to 180), and the Drive Motor
   * Direction if the Drive Motors were reversed.
   * @param  Le_Deg_RotAngCalc (double: caddy rotation calculated angle desired ) 
   * @return Le_Deg_RotAngTgt  (double: caddy rotation target angle requested)
   */
  public double calcRotTgtAngAbs(double Le_Deg_RotAngCalc) {
    double Le_Deg_RotAngTgt;
    Le_Deg_RotAngTgt = (Le_Deg_RotAngCalc - 180);  // Convert from [0 to 360] back to [-180 to 180]
    if (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd) {
        Le_Deg_RotAngTgt = Le_Deg_RotAngTgt + 180;
    }
    return (Le_Deg_RotAngTgt);
  }


  /**
   * Method: calcRotEncdrTgt - Swerve Drive System: Calculates the Rotational
   * Motor Encoder Target Angle in Absolute Encoder Revolutions. Converts the
   * Target Angle from Degrees of Caddy Position to Absolute Encoder Revolutions.
   * 
   * @param Le_Deg_RotAngTgt (double: desired caddy target angle)
   * @return Le_r_RotAngTgt (double: desired rotation motor encoder target)
   */
  public double calcRotEncdrTgt(double Le_Deg_RotAngTgt) {
    double Le_r_RotAngTgt,  Le_r_RotEncdrTgt;
    Le_r_RotAngTgt = (Le_Deg_RotAngTgt / 360);
    Le_r_RotEncdrTgt = Le_r_RotAngTgt * K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat;
    return (Le_r_RotEncdrTgt);
  }


  /**
   * Method: correctRotEncdrTgtPstn - Swerve Drive System: Adjusts the Rotation Control
   * Motor Target Encoder Position with the Zero Position Offset Correction.
   * 
   * @param Le_r_RotEncdrTgt (double: desired caddy target encoder position)
   * @return Le_r_RotEncdrTgtCorr (double: desired caddy target encoder position corrected)
   */
  public double correctRotEncdrTgtPstn(final double Le_r_RotEncdrTgt) {
    double Le_r_RotEncdrTgtCorr;
    Le_r_RotEncdrTgtCorr = (Le_r_RotEncdrTgt + Me_r_RotEncdrZeroOfst);

    return (Le_r_RotEncdrTgtCorr);
  }


  /**
   * Method: setRotEncdrTgt - Swerve Drive System: Sets the Rotational Motor
   * Encoder Target Angle in Absolute Encoder Revolutions. Converts the Target
   * Angle from Degrees of Caddy Position to Absolute Encoder Revolutions.
   * 
   * @param Le_r_RotEncdrTgt (double: desired rotation control encoder target)
   */
  public void setRotEncdrTgt(final double Le_r_RotEncdrTgt) {
    Ms_h_RotMtr.getPIDController().setReference(Le_r_RotEncdrTgt, ControlType.kPosition);
  }


  public void resetRotEncdr() {
    Ms_h_RotEncdr.setPosition(0);
  }



  public double getRotAngActRaw() {
    return(Me_Deg_RotAngActRaw);
  }


  public int getRotModRevs() {
    return(Me_r_RotModRevs);
  }


  /**
   * Method: setRotMtrPwr - Swerve Drive System: Sets the Rotational Motor Speed
   * Pwr Target.
   * 
   * @param Le_r_RotMtrPwr (double: desired rotation motor speed normalized power)
   */
  public void setRotMtrPwr(final double Le_r_RotMtrPwr) {
    Ms_h_RotMtr.getPIDController().setReference(Le_r_RotMtrPwr, ControlType.kDutyCycle);
  }
 

  /**
   * Method: getRotEncdrZeroDtctd - Swerve Drive System: Returns the state
   * of the Rotation Motor Encoder Zero Position Detection Sensor.
   * @return Le_b_ZeroPstnDetect   (boolean: Rotation Zero Position Switch State - Inverted)
   */
	public boolean getRotEncdrZeroDtctd() {
    boolean Le_b_ZeroPstnDetect;  // Presently a False indicates a Zero Position Detection.
    Le_b_ZeroPstnDetect = !(Ms_b_RotZeroDtcd.get());  // So Invert Digital Input
    return(Le_b_ZeroPstnDetect);
  }

  /**
   * Method: getRotEncdrZeroOfst - Swerve Drive System: Returns the Rotational
   * Motor Encoder Target Angle Zero Offset in Swerve Caddy Degrees of Angle.
   */
  public double getRotEncdrZeroOfst() {
    return(Me_r_RotEncdrZeroOfst);
  }

  /**
   * Method: resetRotAngZeroOfst - Swerve Drive System: Resets the Rotational
   * Motor Encoder Target Angle Zero Offset in Swerve Caddy Degrees of Angle.
   */
  public void resetRotEncdrZeroOfst() {
    Me_r_RotEncdrZeroOfst = Ms_h_RotEncdr.getPosition();
  }


  /**
   * Method: learnRotEncdrZeroOfst - Swerve Drive System: Learns the Rotational
   * Motor Encoder Position Zero Offset by sweeping the caddy rotation until
   * the Zero Position Sensor is tripped for a duration of time.
   */
  public Te_RZL_St learnRotEncdrZeroOfst(boolean Le_b_RLZ_InitTrig) {

    /* Initialize Process */
    if (Le_b_RLZ_InitTrig == true) {
      Me_e_RZL_St = Te_RZL_St.Init;
    }

    if (Me_e_RZL_St == Te_RZL_St.Inactv)  {
      /* Do Nothing - Skip to End to Exit */
    }
    /* **Init State** */
    else if (Me_e_RZL_St == Te_RZL_St.Init)  {
      Me_Deg_RZL_AngInit = getRotAngActRaw();
      Me_Deg_RZL_AngSwp = K_SWRV.KeSWRV_Deg_RZL_AngSwpCourse;
      Me_Deg_RZL_AngTgt = Me_Deg_RZL_AngInit + Me_Deg_RZL_AngSwp;
      Me_r_RZL_Pwr = K_SWRV.KeSWRV_r_RZL_PwrSwpCourse;
      Me_t_RZL_DtctTmrIntrsv.reset(); 

      /* **Zero Position Sensor Detected - Goto Zero Detected State** */
      if (getRotEncdrZeroDtctd() == true) {
        Me_e_RZL_St = Te_RZL_St.ZeroDtct;
        sweepCaddyToAng(TeRotDirctn.CW, 0.0);
        Me_t_RZL_DtctTmrIntrsv.start();
      }
      else {
      Me_e_RZL_St = Te_RZL_St.SwpCW;
      sweepCaddyToAng(TeRotDirctn.CW, Me_r_RZL_Pwr);
      }
    }
    /* **Sweep CW State** */
    else if (Me_e_RZL_St == Te_RZL_St.SwpCW) {
      /* **Zero Position Sensor Detected - Goto Zero Detected State** */
      if (getRotEncdrZeroDtctd() == true) {
        Me_e_RZL_St = Te_RZL_St.ZeroDtct;
        sweepCaddyToAng(TeRotDirctn.CW, 0.0);
        Me_t_RZL_DtctTmrIntrsv.start();
      }
      else if (getRotAngActRaw() >= Me_Deg_RZL_AngTgt) {
        Me_e_RZL_St = Te_RZL_St.SwpCCW;
        Me_Deg_RZL_AngTgt = Me_Deg_RZL_AngInit - Me_Deg_RZL_AngSwp;
        sweepCaddyToAng(TeRotDirctn.CCW, Me_r_RZL_Pwr);
      }
      else {
        sweepCaddyToAng(TeRotDirctn.CW, Me_r_RZL_Pwr);
      }    
    }
    /* **Sweep CCW State** */
    else if (Me_e_RZL_St == Te_RZL_St.SwpCCW) {
      /* **Zero Position Sensor Detected - Goto Zero Detected State** */
      if (getRotEncdrZeroDtctd() == true) {
        Me_e_RZL_St = Te_RZL_St.ZeroDtct;
        sweepCaddyToAng(TeRotDirctn.CCW, 0.0);
        Me_t_RZL_DtctTmrIntrsv.start();
      }
      else if (getRotAngActRaw() <= Me_Deg_RZL_AngTgt) {
        Me_e_RZL_St = Te_RZL_St.SwpCW;
        Me_Deg_RZL_AngTgt = Me_Deg_RZL_AngInit;
        sweepCaddyToAng(TeRotDirctn.CW, Me_r_RZL_Pwr);
      }
      else {
        sweepCaddyToAng(TeRotDirctn.CCW, Me_r_RZL_Pwr);
      }
    }
    /* **Zero Detect State** */
    else if (Me_e_RZL_St == Te_RZL_St.ZeroDtct) {
      /* **Zero Position Sensor Still Detected - Goto Zero Detected State** */
      if (getRotEncdrZeroDtctd() == true) {
        if(Me_t_RZL_DtctTmrIntrsv.get() > K_SWRV.KeSWRV_t_RZL_ZeroDtctThrshIntrsv) {
          Me_e_RZL_St = Te_RZL_St.ZeroCptr;          
        }
        else {
          sweepCaddyToAng(TeRotDirctn.CW, 0.0);
        }
      }
      else{
        /* **Zero Position Sensor No Longer Detected - Start Short Sweep Again Slower** */
        Me_Deg_RZL_AngInit = getRotAngActRaw();
        Me_Deg_RZL_AngSwp = K_SWRV.KeSWRV_Deg_RZL_AngSwpFine;
        Me_Deg_RZL_AngTgt = Me_Deg_RZL_AngInit + Me_Deg_RZL_AngSwp;
        Me_r_RZL_Pwr = K_SWRV.KeSWRV_r_RZL_PwrSwpFine;
        Me_e_RZL_St = Te_RZL_St.SwpCW;
      }   
    }    
    /* **Zero Captured State** */
    else if (Me_e_RZL_St == Te_RZL_St.ZeroCptr) {
      resetRotEncdrZeroOfst();
      Me_e_RZL_St = Te_RZL_St.Cmplt;
    }
    /* **Zero Captured State** */
    else if (Me_e_RZL_St == Te_RZL_St.Cmplt) {
      /* Remain Complete until De-Activated */
      Me_e_RZL_St = Te_RZL_St.Cmplt;
    }

  return (Me_e_RZL_St);
  }



/**
  * Method: sweepCaddyToAng - 
  *
  * @param
  * @param
  */
public void sweepCaddyToAng(TeRotDirctn Le_e_RotDirctn, double Le_r_PwrLvl) {
  double Le_r_RotDirctnSclr = 1;

  if (Le_e_RotDirctn == TeRotDirctn.CCW)
    Le_r_RotDirctnSclr = -1;
  setRotMtrPwr(Le_r_PwrLvl * Le_r_RotDirctnSclr);
} {

}



public void montrRotEncdrZeroOfst() {  
  if (getRotEncdrZeroDtctd() == true) {
    Me_t_RZL_DtctTmrPassive.start();
  }
  else {
    Me_t_RZL_DtctTmrPassive.stop();
    Me_t_RZL_DtctTmrPassive.reset();
  }

  if (Me_t_RZL_DtctTmrPassive.get() >= K_SWRV.KeSWRV_t_RZL_ZeroDtctThrshIntrsv) {
    resetRotEncdrZeroOfst();
    Me_Cnt_RZL_PassiveUpd += 1;
  }
}


public Te_RZL_St getRotEncdrZeroLearnSt() {
  return (Me_e_RZL_St);
}


public void setRotEncdrZeroLearnSt(Te_RZL_St Le_e_RZL_St) {
  Me_e_RZL_St = Le_e_RZL_St;
}


public int getRotEncdrZeroLearnCntr() {
  return (Me_Cnt_RZL_PassiveUpd);
}





  /********************************************************/
  /* Drive Control Motor/Encoder Interfaces */
  /********************************************************/

  /**
   * Method: setDrvMtrPwr - Swerve Drive System: Sets the Drive Motor Speed Pwr
   * Target.
   * 
   * @param Le_r_DrvMtrPwr (double: desired drive motor speed normalized power)
   */
  public void setDrvMtrPwr(final double Le_r_RotMtrPwr) {
    Ms_h_DrvMtr.set(ControlMode.PercentOutput, Le_r_RotMtrPwr);
  }


  /**
    * Method: getDriveInverted - Swerve Drive System: Sets the direction of the Drive Motor Speed
    * direction.
    * As of 01/06/20 this is never called, so all driveInverted remain in default, false state.
    * @return  Me_b_DrvMtrInverted   ( boolean: Drive Motor Speed Direction Inverted )
    */
  public boolean getDriveInverted() {
    return(Me_b_DrvMtrInverted);
  }


  /**
    * Method: setDriveInverted - Swerve Drive System: Gets the direction of the Drive Motor Speed
    * direction.
    * As of 01/06/20 this is never called, so all driveInverted remain in default, false state.
    * @param Le_b_DrvMtrInverted   ( boolean: Drive Motor Speed Direction Inverted )
    */
    public void setDriveInverted(boolean Le_b_DrvMtrInverted) {
      Me_b_DrvMtrInverted = Le_b_DrvMtrInverted;
    }



  public void resetDrvEncdrPstn() {
    Ms_h_DrvMtr.setSelectedSensorPosition(0);
  }


  public void resetDrvZeroPstn() {
    Me_Cnt_DrvEncdrZeroPstn = (double)Ms_h_DrvMtr.getSelectedSensorPosition();
  }


  public double getDrvEncdrCntsAbs() {
    return ((double)Ms_h_DrvMtr.getSelectedSensorPosition());
  }


  public double getDrvEncdrCntsRel() {
    return ((double)Ms_h_DrvMtr.getSelectedSensorPosition() - Me_Cnt_DrvEncdrZeroPstn);
  }


  public double getDrvInchesPerEncdrCnts(double Le_Cnts_DrvEncdrCnts, DriveShiftPos Le_e_SelectedGear) {
    double Le_r_DrvGearRat;
    double Le_r_WhlRevs;
    double Le_l_DrvWhlDistInches;

    if (Le_e_SelectedGear == DriveShiftPos.HIGH_GEAR) {
      Le_r_DrvGearRat = K_SWRV.KeSWRV_r_DrvMtrEncdrToWhlRatHi;
    }
    else {
      Le_r_DrvGearRat = K_SWRV.KeSWRV_r_DrvMtrEncdrToWhlRatLo;
    }

    Le_r_WhlRevs = Le_Cnts_DrvEncdrCnts / (K_SWRV.KeSWRV_Cnt_DrvEncdrCntsPerRev * Le_r_DrvGearRat);
    Le_l_DrvWhlDistInches = Le_r_WhlRevs * K_SWRV.KeSWRV_l_DrvWhlDistPerRev;

    return (Le_l_DrvWhlDistInches);
  }


  public int getDrvEncdrCntsPerInches(double Le_l_DrvWhlDistInches, DriveShiftPos Le_e_SelectedGear) {
    double Le_r_DrvGearRat;
    double Le_r_WhlRevs;
    double Le_Cnts_DrvEncdr;

    if (Le_e_SelectedGear == DriveShiftPos.HIGH_GEAR) {
      Le_r_DrvGearRat = K_SWRV.KeSWRV_r_DrvMtrEncdrToWhlRatHi;
    }
    else {
      Le_r_DrvGearRat = K_SWRV.KeSWRV_r_DrvMtrEncdrToWhlRatLo;
    }

    Le_r_WhlRevs = Le_l_DrvWhlDistInches / K_SWRV.KeSWRV_l_DrvWhlDistPerRev;
    Le_Cnts_DrvEncdr = Le_r_WhlRevs * Le_r_DrvGearRat * K_SWRV.KeSWRV_Cnt_DrvEncdrCntsPerRev;
  
    return ((int) Math.round(Le_Cnts_DrvEncdr));
  }

  public double getDrvDistTravelled(DriveShiftPos Le_e_SelectedGear) { 
    double Le_Cnts_DrvEncdrCntDelt;  
    Le_Cnts_DrvEncdrCntDelt = (double)Ms_h_DrvMtr.getSelectedSensorPosition() - Me_Cnt_DrvEncdrZeroPstn;
    return (getDrvInchesPerEncdrCnts(Le_Cnts_DrvEncdrCntDelt, Le_e_SelectedGear));
  }


  public double getDrvDistance(DriveShiftPos Le_e_SelectedGear) {
    double Le_Cnts_DrvEncdrCnt = Ms_h_DrvMtr.getSelectedSensorPosition();
    if (Me_b_DrvMtrInverted == true)
      Le_Cnts_DrvEncdrCnt = -Le_Cnts_DrvEncdrCnt;
    return getDrvInchesPerEncdrCnts(Le_Cnts_DrvEncdrCnt, Le_e_SelectedGear);
}



  /********************************************************/
  /* Drive Motor Direction Interfaces */
  /********************************************************/

  /**
   * Method: getDrvMtrDirctn - Swerve Drive System: Returns the present Direction
   * State setting of the Drive Motor.
   */
  public TeMtrDirctn getDrvMtrDirctn() {
    return (Me_e_DrvMtrDirctn);
  }

  /**
   * Method: invertDrvMtrDirctn - Swerve Drive System - Inverts the Indicator of
   * the Direction of the Drive Motor.
   */
  public void invertDrvMtrDirctn() {
    if (Me_e_DrvMtrDirctn == TeMtrDirctn.Fwd) {
      Me_e_DrvMtrDirctn = TeMtrDirctn.Rwd;
    } else /* (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd) */ {
      Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
    }
  }

  /**
   * Method: setDrvMtrDirctnUpdInhb - Swerve Drive System: Sets the flag that will
   * inhibit the update of the Drive Motor Direction Update
   * 
   * @param Le_b_DrvMtrDirctnUpdInhb (boolean: Drine Motor Direction Update
   *                                 Inhibit)
   */
  public void setDrvMtrDirctnUpdInhb(final boolean Le_b_DrvMtrDirctnUpdInhb) {
    Me_b_DrvMtrDirctnUpdInhb = Le_b_DrvMtrDirctnUpdInhb;
  }

  /**
   * Method: getDrvMtrDirctnUpdInhb - Swerve Drive System: Returns the flag that
   * will inhibit the update of the Drive Motor Direction Update.
   * 
   * @return Me_b_DrvMtrDirctnUpdInhb (boolean: Drive Motor Direction Update
   *         Inhibit)
   */
  public boolean getDrvMtrDirctnUpdInhb() {
    return Me_b_DrvMtrDirctnUpdInhb;
  }

  /**
   * Method: getDrvMtrDirctnTrig - Swerve Drive System: Sets the indication that
   * the the Direction of the Drive Motor is being inverted.
   * 
   */
  public boolean getDrvMtrDirctnTrig() {
      return  Me_b_DrvMtrDirctnUpdTrig;
    }
  /**
   * Method: setDrvMtrDirctnTrig - Swerve Drive System: Sets the indication that
   * the the Direction of the Drive Motor is being inverted.
   * 
   */
  public void setDrvMtrDirctnTrig(final boolean Le_b_DrvMtrDirctnTrig) {
    Me_b_DrvMtrDirctnUpdTrig = Le_b_DrvMtrDirctnTrig;
  }

}
