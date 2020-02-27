/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveSubsystem.TeRotDirctn;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SDRV_RotFindZero extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  SwerveDriveSubsystem swerveDriveSubsystem;

	public enum TeCmndSt {
    Init,
    SwpCW,
    SwpCCW,
    ZeroDtct,
    ZeroCptr,
    End;
  }

  int      Xe_i_ModIdx;
  TeCmndSt Xe_e_CmndSt;
  double   Xe_Deg_RotAngInit;
  double   Xe_Deg_RotAngSwp;
  double   Xe_Deg_RotAngTgt;
  double   Xe_r_RotPwr;
  Timer    Xe_t_ZeroDtctTmr = new Timer();
  double   Xe_t_ZeroDtctThrsh;
 
  public SDRV_RotFindZero(SwerveDriveSubsystem swerveDriveSubsystem, int Xe_i_ModIdx) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.Xe_i_ModIdx = Xe_i_ModIdx;
    addRequirements(this.swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Xe_e_CmndSt = TeCmndSt.Init; 
    Xe_Deg_RotAngInit = swerveDriveSubsystem.getSwerveCaddyAng(Xe_i_ModIdx);
    Xe_Deg_RotAngSwp = 270;
    Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit + Xe_Deg_RotAngSwp;
    Xe_r_RotPwr = 0.15;
    Xe_t_ZeroDtctThrsh = 0.250;
    Xe_t_ZeroDtctTmr.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    /* **Init State** */
    if (Xe_e_CmndSt == TeCmndSt.Init)  {
        /* **Zero Position Sensor Detected - Goto Zero Detected State** */
        if (swerveDriveSubsystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
          Xe_e_CmndSt = TeCmndSt.ZeroDtct;
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
          Xe_t_ZeroDtctTmr.start();
        }
        else {
        Xe_e_CmndSt = TeCmndSt.SwpCW;
        swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
        }
    }
    /* **Sweep CW State** */
    else if (Xe_e_CmndSt == TeCmndSt.SwpCW) {
        /* **Zero Position Sensor Detected - Goto Zero Detected State** */
        if (swerveDriveSubsystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
          Xe_e_CmndSt = TeCmndSt.ZeroDtct;
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
          Xe_t_ZeroDtctTmr.start();
        }
        else if (swerveDriveSubsystem.getSwerveCaddyAng(Xe_i_ModIdx) >= Xe_Deg_RotAngTgt) {
          Xe_e_CmndSt = TeCmndSt.SwpCCW;
          Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit - Xe_Deg_RotAngSwp;
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CCW, Xe_r_RotPwr);
        }
        else {
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
        }    
    }
    /* **Sweep CCW State** */
    else if (Xe_e_CmndSt == TeCmndSt.SwpCCW) {
        /* **Zero Position Sensor Detected - Goto Zero Detected State** */
        if (swerveDriveSubsystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
          Xe_e_CmndSt = TeCmndSt.ZeroDtct;
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
          Xe_t_ZeroDtctTmr.start();
        }
        else if (swerveDriveSubsystem.getSwerveCaddyAng(Xe_i_ModIdx) <= Xe_Deg_RotAngTgt) {
          Xe_e_CmndSt = TeCmndSt.SwpCW;
          Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit;
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
        }
        else {
          swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CCW, Xe_r_RotPwr);
        }
    }
    /* **Zero Detect State** */
    else if (Xe_e_CmndSt == TeCmndSt.ZeroDtct) {
        /* **Zero Position Sensor Still Detected - Goto Zero Detected State** */
        if (swerveDriveSubsystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
          if(Xe_t_ZeroDtctTmr.get() >= Xe_t_ZeroDtctThrsh) {
            Xe_e_CmndSt = TeCmndSt.ZeroCptr;          
          }
          else {
            swerveDriveSubsystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
          }
        }
        else{
          /* **Zero Position Sensor No Longer Detected - Start Short Sweep Again Slower** */
          Xe_Deg_RotAngInit = swerveDriveSubsystem.getSwerveCaddyAng(Xe_i_ModIdx);
          Xe_Deg_RotAngSwp = 10.0;
          Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit + Xe_Deg_RotAngSwp;
          Xe_r_RotPwr = 0.05;
          Xe_e_CmndSt = TeCmndSt.SwpCW;
        }   
    }    
    /* **Zero Captured State** */
    else if (Xe_e_CmndSt == TeCmndSt.ZeroCptr) {
      swerveDriveSubsystem.resetCaddyRotEncdr(Xe_i_ModIdx);
      Xe_e_CmndSt = TeCmndSt.End;
    }
    /* **Zero Captured State** */
    else if (Xe_e_CmndSt == TeCmndSt.End) {
      /* Do nothing - The End */
    }

  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Xe_t_ZeroDtctTmr.stop();
    swerveDriveSubsystem.haltSwerveDrive();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean Le_b_EndCond = false;
    if (Xe_e_CmndSt == TeCmndSt.End) {
      Le_b_EndCond = true;
    }

    return Le_b_EndCond;
  }
}
