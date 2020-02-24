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

    private double Me_r_ModRevs;

    private final double Me_Deg_RotEncdrZeroOfst;
    private double Me_t_RotMtrStlInitTm;
 
    private TeMtrDirctn Me_e_DrvMtrDirctn;
    private boolean Me_b_DrvMtrDirctnUpdInhb;
    private boolean Me_b_DrvMtrDirctnUpdTrig;


	SwerveDriveModule(int Le_i_ModIdx, CANSparkMax Ls_h_RotMtr, double Le_Deg_RotZeroOfst) {
        Me_i_ModIdx = Le_i_ModIdx;
        Me_r_ModRevs = 0;
        
		    Ms_h_RotMtr = Ls_h_RotMtr;
        Ms_h_RotEncdr = Ms_h_RotMtr.getEncoder();

        Me_Deg_RotEncdrZeroOfst = Le_Deg_RotZeroOfst;
        Me_t_RotMtrStlInitTm = 0;

        Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
        Me_b_DrvMtrDirctnUpdInhb = false;
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

    }
	

    public CANSparkMax getRotMtr() {
        return Ms_h_RotMtr;
    }


    public void disableRobotInitMd() {
        Me_t_RotMtrStlInitTm = Long.MAX_VALUE;
    }


   /** Method: getRotEncdrActPstn - Swerve Drive System - Gets the Rotational
      * Motor Actual Feedback Angle that has Zero Offset and Drive
      * Motor Correction applied.  
      * @return Le_r_RotEncdrActPstn (double: caddy rotation motor encoder nominal position)	
      */	  
      public double getRotEncdrActPstn() {
        double Le_r_RotEncdrActPstn = Ms_h_RotEncdr.getPosition();
        return Le_r_RotEncdrActPstn;
    }


    /** Method: getRotActAngRaw - Swerve Drive System - Gets the Rotational
      * Motor Actual Feedback Angle that has not been normalizec for
      * a single caddy rotation.  
      * @return Le_Deg_RotAngAct (double: caddy rotation actual angle)	
      */	  
    public double getRotActAngRaw() {
        double Le_Deg_RotAngActRaw; 

        Le_Deg_RotAngActRaw = Ms_h_RotEncdr.getPosition() * (360.0 / K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat);

        return Le_Deg_RotAngActRaw;
    }


    /** Method: cnvrtRotActAng - Swerve Drive System - Gets the Rotational
      * Motor Actual Feedback Angle that has Zero Offset and Drive
      * Motor Correction applied.  
      * @return Le_Deg_RotAngAct (double: caddy rotation actual angle)	
      */	  
      public double cnvrtRotActAng(double Le_Deg_RotAngActRaw) {
        double Le_Deg_RotAngActClpd, Le_Deg_RotAngAct; 

        Me_r_ModRevs =  (int)(Le_Deg_RotAngActRaw/360);        

        Le_Deg_RotAngActClpd = Le_Deg_RotAngActRaw % 360;

        if (Le_Deg_RotAngActClpd < 0) {
            Le_Deg_RotAngAct = 360 -  Le_Deg_RotAngActClpd;
        }
        else {
            Le_Deg_RotAngAct = Le_Deg_RotAngActClpd;
        } 

        return Le_Deg_RotAngAct;
    }



    /** Method: adjustRotTgtAng - Swerve Drive System: Adjusts the
      * Rotation Control Motor Target Angle with the Zero Position
      * Offset Correction and Correction for Drive Motor Direction
      * if the Drive Motor is Inverted.
      * @param Le_Deg_RotAngTgtRaw (double: raw desired caddy target angle)
      * @return Le_Deg_RotAngTgtCorr (double: corrected desired caddy target angle)	
      */	  
    public double adjustRotTgtAng(double Le_Deg_RotAngTgtRaw) {
        double Le_Deg_RotAngTgtTemp, Le_Deg_RotAngTgtCorr;

        Le_Deg_RotAngTgtTemp = Le_Deg_RotAngTgtRaw + Me_Deg_RotEncdrZeroOfst;

        if (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd) {
            Le_Deg_RotAngTgtTemp = Le_Deg_RotAngTgtTemp + 180;
        }

        Le_Deg_RotAngTgtCorr = Le_Deg_RotAngTgtTemp % 360;
        
        return(Le_Deg_RotAngTgtCorr);
    }


    /** Method: calcRotEncdrTgt - Swerve Drive System: Calculates the
      * Rotational Motor Encoder Target Angle in Absolute Encoder
      * Revolutions. Converts the Target Angle from Degrees of Caddy
      * Position to Absolute Encoder Revolutions.  
      * @param Le_Deg_RotAngTgt (double: desired caddy target angle)
      * @return Le_r_RotAngTgt 	(double: desired rotation motor encoder target)
      */	  
    public double calcRotEncdrTgt(double Le_Deg_RotAngTgt) {
        double Le_r_RotAngTgt, Le_r_RotAngTgtMtr, Le_r_RotAngMtrWndUpOfst;
        double Le_r_RotEncdrTgt;
        Le_r_RotAngTgt = (Le_Deg_RotAngTgt/360);
        Le_r_RotAngTgtMtr = Le_r_RotAngTgt * K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat;

        Le_r_RotAngMtrWndUpOfst = Me_r_ModRevs * K_SWRV.KeSWRV_r_RotMtrEncdrToCaddyRat;

        Le_r_RotEncdrTgt = Le_r_RotAngTgtMtr + Le_r_RotAngMtrWndUpOfst;

        return (Le_r_RotEncdrTgt);
    }


    /** Method: setRotEncdrTgt - Swerve Drive System: Sets the Rotational
      * Motor Encoder Target Angle in Absolute Encoder Revolutions. Converts
      * the Target Angle from Degrees of Caddy Position to Absolute Encoder
      * Revolutions.  
      * @param Le_r_RotEncdrTgt (double: desired rotation control encoder target)
      */
    public void setRotEncdrTgt(double Le_r_RotEncdrTgt) {
        Ms_h_RotMtr.getPIDController().setReference(Le_r_RotEncdrTgt, ControlType.kPosition);
    }


    public void resetRotEncdr() {
        Ms_h_RotEncdr.setPosition(0);
    }


    /** Method: getDrvMtrDirctn - Swerve Drive System: Returns the
      * present Direction State setting of the Drive Motor. 
      */   
    public TeMtrDirctn getDrvMtrDirctn()  {
        return(Me_e_DrvMtrDirctn);
    }

    
    /** Method: invertDrvMtrDirctn - Swerve Drive System - Inverts the
      * Indicator of the Direction of the Drive Motor. 
      */   
    public void invertDrvMtrDirctn()  {
        if (Me_e_DrvMtrDirctn == TeMtrDirctn.Fwd) {
          Me_e_DrvMtrDirctn = TeMtrDirctn.Rwd;
        }
        else /* (Me_e_DrvMtrDirctn == TeMtrDirctn.Rwd) */ {
          Me_e_DrvMtrDirctn = TeMtrDirctn.Fwd;
        }
    }


    /** Method: setDrvMtrDirctnUpdInhb - Swerve Drive System: Sets
      * the flag that will inhibit the update of the Drive Motor
      * Direction Update
      * @param Le_b_DrvMtrDirctnUpdInhb (boolean: Drine Motor Direction Update Inhibit)
      */   
      public void setDrvMtrDirctnUpdInhb(boolean Le_b_DrvMtrDirctnUpdInhb)  {
        Me_b_DrvMtrDirctnUpdInhb = Le_b_DrvMtrDirctnUpdInhb;
    }


    /** Method: getDrvMtrDirctnUpdInhb - Swerve Drive System: Returns
      * the flag that will inhibit the update of the Drive Motor
      * Direction Update.
      * @return Me_b_DrvMtrDirctnUpdInhb (boolean: Drive Motor Direction Update Inhibit)
      */
      public boolean getDrvMtrDirctnUpdInhb()  {
        return Me_b_DrvMtrDirctnUpdInhb;
    }


    /** Method: setDrvMtrDirctnTrig - Swerve Drive System: Sets
      * the indication that the the Direction of the Drive Motor
      * is being inverted.
      *  
      */   
    public void setDrvMtrDirctnTrig(boolean Le_b_DrvMtrDirctnTrig)  {
        Me_b_DrvMtrDirctnUpdTrig = Le_b_DrvMtrDirctnTrig;
    }


}
