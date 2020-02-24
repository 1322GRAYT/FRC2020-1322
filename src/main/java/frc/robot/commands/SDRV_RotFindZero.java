/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.SwrvMap;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;
import frc.robot.subsystems.SwerveDriveSubsystem.TeRotDirctn;

import javax.lang.model.util.ElementScanner6;

import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SDRV_RotFindZero extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  SwerveDriveSubsystem swerveDriveSystem;

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
  double   Xe_Deg_RotAngTgt;
  double   Xe_r_RotPwr;
  Timer    Xe_t_ZeroDtctTmr = new Timer();
  double   Xe_t_ZeroDtctThrsh;
 
  public SDRV_RotFindZero(SwerveDriveSubsystem swerveDriveSystem, int Xe_i_ModIdx) {
    this.swerveDriveSystem = swerveDriveSystem;
    this.Xe_i_ModIdx = Xe_i_ModIdx;
    addRequirements(this.swerveDriveSystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Xe_e_CmndSt = TeCmndSt.Init; 
    Xe_Deg_RotAngInit = swerveDriveSystem.getSwerveCaddyAng(Xe_i_ModIdx);
    Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit + 45;
    Xe_r_RotPwr = 0.10;
    Xe_t_ZeroDtctThrsh = 0.250;
    Xe_t_ZeroDtctTmr.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if (swerveDriveSystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
      Xe_e_CmndSt = TeCmndSt.ZeroDtct;
      swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
      Xe_t_ZeroDtctTmr.start();
    }
    else if (Xe_e_CmndSt == TeCmndSt.ZeroDtct) {
      if (swerveDriveSystem.getRotZeroDtctd(Xe_i_ModIdx) == false) {
        if(Xe_t_ZeroDtctTmr.get() >= Xe_t_ZeroDtctThrsh) {
          Xe_e_CmndSt = TeCmndSt.ZeroCptr;          
        }
        else {
          swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, 0.0);
        }
      }
      else{
        Xe_Deg_RotAngInit = swerveDriveSystem.getSwerveCaddyAng(Xe_i_ModIdx);
        Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit + 10;
        Xe_r_RotPwr = 0.05;
        Xe_e_CmndSt = TeCmndSt.SwpCW;
      }   
    }    
    else if (Xe_e_CmndSt == TeCmndSt.Init)  {
      Xe_e_CmndSt = TeCmndSt.SwpCW;
      swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
    }
    else if (Xe_e_CmndSt == TeCmndSt.SwpCW) {
      if (swerveDriveSystem.getSwerveCaddyAng(Xe_i_ModIdx) >= Xe_Deg_RotAngTgt) {
        Xe_e_CmndSt = TeCmndSt.SwpCCW;
        Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit - 25;
        swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CCW, Xe_r_RotPwr);
      }
      else {
        swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
      }    
    }
    else if (Xe_e_CmndSt == TeCmndSt.SwpCCW) {
      if (swerveDriveSystem.getSwerveCaddyAng(Xe_i_ModIdx) <= Xe_Deg_RotAngTgt) {
        Xe_e_CmndSt = TeCmndSt.SwpCW;
        Xe_Deg_RotAngTgt = Xe_Deg_RotAngInit;
        swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CW, Xe_r_RotPwr);
      }
      else {
        swerveDriveSystem.sweepSwerveCaddyToAng(Xe_i_ModIdx, TeRotDirctn.CCW, Xe_r_RotPwr);
      }    
    }
    else if (Xe_e_CmndSt == TeCmndSt.ZeroCptr) {
      Xe_e_CmndSt = TeCmndSt.End;
    }    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Xe_t_ZeroDtctTmr.stop();
    swerveDriveSystem.haltSwerveDrive();
    swerveDriveSystem.resetCaddyRotEncdr(Xe_i_ModIdx);
    swerveDriveSystem.resetCaddyRotZeroOfst(Xe_i_ModIdx);
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
