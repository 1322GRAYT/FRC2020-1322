/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.calibrations.K_DRV;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRV_RotToAngAbs extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final DriveSubsystem driveSubsystem;
  private double Xe_r_RotPwr;
  private final double Xe_Deg_RotAngTgt;
  private double Xe_Deg_RotAngErr;
  private boolean Xe_b_WithinDB;
  private boolean Xe_b_SyncTmrStrtd = false;
  private Timer Xe_t_SyncTmr = new Timer();


  public DRV_RotToAngAbs(DriveSubsystem driveSubsystem, double Le_Deg_RotAngDsrd) {
    this.driveSubsystem = driveSubsystem;
    Xe_Deg_RotAngTgt = Le_Deg_RotAngDsrd;
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Xe_t_SyncTmr.stop();
      Xe_t_SyncTmr.reset();
      Xe_b_SyncTmrStrtd = false;
      
      driveSubsystem.resetRotPID();
      driveSubsystem.configPID_RotCtrl(Xe_Deg_RotAngTgt);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {	  
      Xe_Deg_RotAngErr = Xe_Deg_RotAngTgt - driveSubsystem.getGyroAngRaw();
      Xe_r_RotPwr = driveSubsystem.PID_RotCtrl(Xe_Deg_RotAngErr);
      driveSubsystem.setDRV_r_PID_DrvPowCorr(Xe_r_RotPwr);
 
      driveSubsystem.TankDrive(0, Xe_r_RotPwr, 0);

      Xe_b_WithinDB = driveSubsystem.getRotAtSetpoint();        
      if (Xe_b_WithinDB) {
          if (Xe_b_SyncTmrStrtd == false) {
              Xe_t_SyncTmr.start();
              Xe_b_SyncTmrStrtd = true;
          }
      } else {
          Xe_t_SyncTmr.stop();
          Xe_t_SyncTmr.reset();
          Xe_b_SyncTmrStrtd = false;
      }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveSubsystem.stopDrvMtrAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return(Xe_t_SyncTmr.hasPeriodPassed(K_DRV.KeDRV_t_CL_RotSyncThrshRot));
  }

}
