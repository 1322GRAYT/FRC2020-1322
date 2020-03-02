/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DRV_DrvFwdFeet extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final DriveSubsystem driveSubsystem;
  private double Xe_r_LongPwr;
  private double Xe_Deg_HdgAngInit;
  private double Xe_l_DsrdFtLong;
  private double Xe_l_DsrdFtLat;
  private final double Xe_Deg_CalcDrvAng;
  private final double Xe_l_CalcDrvFt;
  private final double Xe_l_CalcDrvInches;

  private boolean isTimerStarted = false;
  private double encoderStart [];

  private double Xe_Cnt_DsrdEncdrCntTgt, Xe_Cnt_DsrdEncdrCntErr;
  private Timer Xe_t_SyncTmr = new Timer();



  public DRV_DrvFwdFeet(DriveSubsystem driveSubsystem, double Le_l_DsrdFtLong, double Le_l_DsrdFtLat) {
    this.driveSubsystem = driveSubsystem;
    Xe_Deg_HdgAngInit = driveSubsystem.getGyroAngRaw();
    Xe_l_DsrdFtLong = Le_l_DsrdFtLong;
    Xe_l_DsrdFtLat = Le_l_DsrdFtLat;
    Xe_Deg_CalcDrvAng = Math.toDegrees(Math.atan2(Xe_l_DsrdFtLat, Xe_l_DsrdFtLong));
    Xe_l_CalcDrvFt = Math.sqrt(Xe_l_DsrdFtLat * Xe_l_DsrdFtLat + Xe_l_DsrdFtLong * Xe_l_DsrdFtLong);
    Xe_l_CalcDrvInches = Xe_l_CalcDrvFt * 12;
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      driveSubsystem.resetDrvEncdrPstnAll();
      Xe_Cnt_DsrdEncdrCntTgt = driveSubsystem.getDrvEncdrCntsPerInches(Xe_l_CalcDrvInches);
      Xe_t_SyncTmr.stop();
      Xe_t_SyncTmr.reset();
         
      driveSubsystem.resetRotPID();
      // Rotation PID (has continuous input)
      driveSubsystem.setRotWraparoundInputRange(0, 360);
      driveSubsystem.setRotSetpoint(Xe_Deg_HdgAngInit + Xe_Deg_CalcDrvAng);
      driveSubsystem.setRotTolerance(18, 18); // +/- 5% of setpoint is OK for pos and vel.
      driveSubsystem.setRotOutputRange(-1, 1);
        
      // Set up forward pid:
      driveSubsystem.setDrvSetpoint(Xe_Cnt_DsrdEncdrCntTgt);
      driveSubsystem.setDrvTolerance(6, 15);
      driveSubsystem.setDrvOutputRange(-1, 1);
  
      isTimerStarted = false;
  
      //initialDrivetrainAngle = m_drivetrain.getGyroAngle();
      for( int i=0; i<4; i++) {
          encoderStart[i] = driveSubsystem.getDriveMotor(i).getSelectedSensorPosition();
      }
  

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Xe_r_LongPwr = 0; // todo - Hook Up

    driveSubsystem.TankDrive(Xe_r_LongPwr, 0, Xe_Deg_HdgAngInit, true);
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
