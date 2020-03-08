/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.calibrations.K_DRV;
import frc.robot.subsystems.RFSLIB;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRV_DrvManual extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command t    driveSubsystem.zeroGyro();o Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final DriveSubsystem driveSubsystem;
  private final XboxController driverStick;
  private final RFSLIB rfsLIB = new RFSLIB();
  private double Xe_r_LongPwr, Xe_r_RotPwr;
  private double Xe_r_RotPwrHdgCorr;
  private double Xe_Deg_HdgAngInit;
  private double Xe_Deg_HdgAngErr;
  private boolean Xe_b_HdgAngUpd;

  public DRV_DrvManual(DriveSubsystem driveSubsystem, XboxController driverStick) {
      this.driveSubsystem = driveSubsystem;
      this.driverStick = driverStick;
      Xe_Deg_HdgAngInit = driveSubsystem.getGyroAngRaw();
      Xe_b_HdgAngUpd = false;
      addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      driveSubsystem.resetRotPID();
      driveSubsystem.configPID_RotCtrlHdgCorr(Xe_Deg_HdgAngInit);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      Xe_r_LongPwr = -driverStick.getY(Hand.kLeft);
      Xe_r_RotPwr  =  driverStick.getX(Hand.kRight);

      Xe_r_LongPwr = rfsLIB.ApplyDB_Clpd(Xe_r_LongPwr, K_DRV.KeDRV_r_DB_CntlrThrshRot);
      Xe_r_RotPwr  = rfsLIB.ApplyDB_Clpd(Xe_r_RotPwr, K_DRV.KeDRV_r_DB_CntlrThrshRot);

      if ((Math.abs(Xe_r_LongPwr) < K_DRV.KeDRV_r_DB_CntlrThrshFwd) &&
          (Math.abs(Xe_r_RotPwr)  < K_DRV.KeDRV_r_DB_CntlrThrshRot)) {
          driveSubsystem.resetRotPID();
          Xe_Deg_HdgAngErr = ((Xe_Deg_HdgAngInit - driveSubsystem.getGyroAngRaw()) / 180)*10;
          Xe_r_RotPwrHdgCorr = 0;
          Xe_b_HdgAngUpd = false;
      }
      else if ((Math.abs(Xe_r_LongPwr) >= K_DRV.KeDRV_r_DrvRqstOvrrdFwd) &&
               (Math.abs(Xe_r_RotPwr)  <  K_DRV.KeDRV_r_DrvRqstOvrrdRot)) {
          if (Xe_b_HdgAngUpd == false) {
              Xe_Deg_HdgAngInit = driveSubsystem.getGyroAngRaw();
              Xe_Deg_HdgAngErr = 0;
              Xe_b_HdgAngUpd = true;
          }
          else { 
              Xe_Deg_HdgAngErr = ((Xe_Deg_HdgAngInit - driveSubsystem.getGyroAngRaw()) / 180)*10;
          }
          Xe_r_RotPwrHdgCorr = driveSubsystem.PID_RotCtrl(Xe_Deg_HdgAngErr); 
      }
      else {
          driveSubsystem.resetRotPID();
          Xe_Deg_HdgAngErr = 0;
          Xe_r_RotPwrHdgCorr = 0;
          Xe_b_HdgAngUpd = false;
      }
      driveSubsystem.setDRV_r_PID_RotPowCorr(Xe_r_RotPwrHdgCorr);
      driveSubsystem.TankDrive(Xe_r_LongPwr, Xe_r_RotPwr, Xe_r_RotPwrHdgCorr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
      driveSubsystem.stopDrvMtrAll();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
      return (false);
  }

}
