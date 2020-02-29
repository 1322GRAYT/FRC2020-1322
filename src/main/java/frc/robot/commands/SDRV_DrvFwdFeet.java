/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.Constants.DriveShiftPos;
import frc.robot.Constants.SolenoidPosition;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.ShiftSubsystem;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SDRV_DrvFwdFeet extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private final SwerveDriveSubsystem swerveDriveSubsystem;
  private final ShiftSubsystem shiftSubsystem;
  private final DriveShiftPos Xe_e_SlctdGr;
  private double Xe_l_DsrdFtLong;
  private double Xe_l_DsrdFtLat;
  private final double Xe_deg_CalcDrvAng;
  private final double Xe_l_CalcDrvFt;
  private final double Xe_l_CalcDrvInches;

  private boolean isTimerStarted = false;
  private double encoderStart [];

  private double Xe_Cnt_DsrdEncdrCntTgt, Xe_Cnt_DsrdEncdrCntErr;
  private Timer Xe_t_SyncTmr = new Timer();

  public SDRV_DrvFwdFeet(SwerveDriveSubsystem swerveDriveSubsystem, ShiftSubsystem shiftSubsystem,
                         double Le_l_DsrdFtLong, double Le_l_DsrdFtLat) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.shiftSubsystem = shiftSubsystem;
    Xe_e_SlctdGr = shiftSubsystem.getSelectedGear();
    Xe_l_DsrdFtLong = Le_l_DsrdFtLong;
    Xe_l_DsrdFtLat = Le_l_DsrdFtLat;
    Xe_deg_CalcDrvAng = Math.toDegrees(Math.atan2(Xe_l_DsrdFtLat, Xe_l_DsrdFtLong));
    Xe_l_CalcDrvFt = Math.sqrt(Xe_l_DsrdFtLat * Xe_l_DsrdFtLat + Xe_l_DsrdFtLong * Xe_l_DsrdFtLong);
    Xe_l_CalcDrvInches = Xe_l_CalcDrvFt * 12;
    addRequirements(this.swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    swerveDriveSubsystem.resetDrvEncdrs();
    Xe_Cnt_DsrdEncdrCntTgt = swerveDriveSubsystem.getDrvCaddyEncdrCntsPerInches(Xe_l_CalcDrvInches, Xe_e_SlctdGr);
    Xe_t_SyncTmr.stop();
    Xe_t_SyncTmr.reset();
       
    swerveDriveSubsystem.resetPID();
    // Rotation PID (has continuous input)
    swerveDriveSubsystem.setRotationWraparoundInputRange(0, 360);
    swerveDriveSubsystem.setRotationSetpoint(swerveDriveSubsystem.getGyroAngle());
    swerveDriveSubsystem.setRotationTolerance(18, 18); // +/- 5% of setpoint is OK for pos and vel.
    swerveDriveSubsystem.setRotationOutputRange(-1, 1);
    
    // Set up strafe pid:
    swerveDriveSubsystem.setStrafeTolerance(6, 16); // +/- 5% of setpoint is OK for pos and vel.
    swerveDriveSubsystem.setStrafeOutputRange(-1, 1);
    swerveDriveSubsystem.setStrafeSetpoint(Xe_l_DsrdFtLat);

    // Set up forward pid:
    swerveDriveSubsystem.setForwardSetpoint(Xe_l_DsrdFtLong);
    swerveDriveSubsystem.setForwardTolerance(6, 15);
    swerveDriveSubsystem.setForwardOutputRange(-1, 1);

    isTimerStarted = false;

    //initialDrivetrainAngle = m_drivetrain.getGyroAngle();
    for( int i=0; i<4; i++) {
        encoderStart[i] = swerveDriveSubsystem.getSwerveModule(i).getDrvDistance(Xe_e_SlctdGr);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {    
    //todo Command Drive
    double forwardFactor = Xe_l_DsrdFtLong / Xe_l_CalcDrvInches;
    double strafeFactor =  Xe_l_DsrdFtLat  / Xe_l_CalcDrvInches;
    double encError = 0;
    for( int i=0; i<4; i++) {
        encError += encoderStart[i] - swerveDriveSubsystem.getSwerveModule(i).getDrvDistance(Xe_e_SlctdGr);
    }
    encError /= 4;
    double fwdErr = encError  * forwardFactor;
    double strafeErr = encError * strafeFactor;

    swerveDriveSubsystem.PID_DrvCntrl(fwdErr, strafeErr, swerveDriveSubsystem.getGyroAngle(), true);
    


    Xe_Cnt_DsrdEncdrCntErr = Xe_Cnt_DsrdEncdrCntTgt - swerveDriveSubsystem.getDrvCaddyEncdrCnts();
    if (Math.abs(Xe_Cnt_DsrdEncdrCntErr) <= K_SWRV.KeSWRV_Cnt_CL_DrvErrTgtDB_Fwd) {
      Xe_t_SyncTmr.start();      
    }
    else {
      Xe_t_SyncTmr.stop();      
      Xe_t_SyncTmr.reset();      
    }
         
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stopDrvMtrs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    boolean inDeadBand = swerveDriveSubsystem.forwardAtSetpoint() && swerveDriveSubsystem.strafeAtSetpoint() && swerveDriveSubsystem.rotationAtSetpoint();
        
    if (inDeadBand) {
      if (!isTimerStarted) {
        Xe_t_SyncTmr.start();
        isTimerStarted = true;
      }
    }
    else {
        Xe_t_SyncTmr.stop();
        Xe_t_SyncTmr.reset();
        isTimerStarted = false;
    }

    return (Xe_t_SyncTmr.hasPeriodPassed(K_SWRV.KeSWRV_t_CL_DrvSyncThrshFwd));
  }
}
