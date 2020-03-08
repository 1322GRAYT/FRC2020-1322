/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.DrvMap;
import frc.robot.calibrations.K_DRV;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRV_DrvFwdFeet extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final DriveSubsystem driveSubsystem;
  private double Xe_r_DrvPwrLong;
  private double Xe_r_RotPwrHdgCorr;
  private final double Xe_Deg_HdgAngInit;
  private final double Xe_Deg_HdgAngCalc;
  private final double Xe_l_DsrdFtLong;
  private final double Xe_l_DsrdFtLat;
  private final double Xe_l_CalcDrvFt;
  private final double Xe_l_CalcDrvInches;
  private int[] Xa_Cnt_DrvEncdrInit = new int[DrvMap.NumOfMtrs];
  private int Xe_Cnt_DrvEncdrCntTgt;
  private int Xe_Cnt_DrvEncdrCntErr;
  private double Xe_Deg_HdgAngTgt;
  private double Xe_Deg_HdgAngErr;
  private boolean Xe_b_WithinDB;
  private boolean Xe_b_SyncTmrStrtd = false;
  private Timer Xe_t_SyncTmr = new Timer();


  public DRV_DrvFwdFeet(DriveSubsystem driveSubsystem, double Le_l_DsrdFtLong, double Le_l_DsrdFtLat) {
    this.driveSubsystem = driveSubsystem;
    Xe_Deg_HdgAngInit = driveSubsystem.getGyroAngRaw();
    Xe_l_DsrdFtLong = Le_l_DsrdFtLong;
    Xe_l_DsrdFtLat = Le_l_DsrdFtLat;
    Xe_Deg_HdgAngCalc = Math.toDegrees(Math.atan2(Xe_l_DsrdFtLat, Xe_l_DsrdFtLong));
    Xe_l_CalcDrvFt = Math.sqrt(Xe_l_DsrdFtLat * Xe_l_DsrdFtLat + Xe_l_DsrdFtLong * Xe_l_DsrdFtLong);
    Xe_l_CalcDrvInches = Xe_l_CalcDrvFt * 12;
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      Xe_t_SyncTmr.stop();
      Xe_t_SyncTmr.reset();
      Xe_b_SyncTmrStrtd = false;
      
      driveSubsystem.resetDrvEncdrPstnAll();
      driveSubsystem.resetDrvPID();
      Xe_Cnt_DrvEncdrCntTgt = driveSubsystem.getDrvEncdrCntsPerInches(Xe_l_CalcDrvInches);     
      driveSubsystem.configPID_DrvCtrl(Xe_Cnt_DrvEncdrCntTgt);
      
      driveSubsystem.resetRotPID();
      Xe_Deg_HdgAngTgt = Xe_Deg_HdgAngInit + Xe_Deg_HdgAngCalc;
      driveSubsystem.configPID_RotCtrlHdgCorr(Xe_Deg_HdgAngTgt);
        
      for( int i=0; i<4; i++) {
          Xa_Cnt_DrvEncdrInit[i] = driveSubsystem.getDriveMotor(i).getSelectedSensorPosition();
      }  
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double Le_Cnt_EncdrCntErr = 0;
      for( int i=0; i<4; i++) {
          Le_Cnt_EncdrCntErr += Xa_Cnt_DrvEncdrInit[i] - driveSubsystem.getDriveMotor(i).getSelectedSensorPosition();
      }
      /* Calculate Average Encoder Count Error from the 4 Drive Encoders */
      Xe_Cnt_DrvEncdrCntErr = (int)Math.round(Le_Cnt_EncdrCntErr/4);
      Xe_r_DrvPwrLong = driveSubsystem.PID_DrvCtrl((double)Xe_Cnt_DrvEncdrCntErr); 	  
      driveSubsystem.setDRV_r_PID_DrvPowCorr(Xe_r_DrvPwrLong);

      Xe_Deg_HdgAngErr = Xe_Deg_HdgAngTgt - driveSubsystem.getGyroAngRaw();
      Xe_r_RotPwrHdgCorr = driveSubsystem.PID_RotCtrl(Xe_Deg_HdgAngErr); 
      driveSubsystem.setDRV_r_PID_RotPowCorr(Xe_r_RotPwrHdgCorr);
     
      driveSubsystem.TankDrive(Xe_r_DrvPwrLong, 0, Xe_r_RotPwrHdgCorr);

      Xe_b_WithinDB = driveSubsystem.getDrvAtSetpoint() && driveSubsystem.getRotAtSetpoint();        
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
      return(Xe_t_SyncTmr.hasPeriodPassed(K_DRV.KeDRV_t_CL_DrvSyncThrshFwd));
  }

}
