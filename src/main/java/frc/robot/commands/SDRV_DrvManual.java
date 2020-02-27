/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import frc.robot.calibrations.K_SWRV;
import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SDRV_DrvManual extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private XboxController driverStick;
  private double Xe_r_LongPwr, Xe_r_LatPwr, Xe_r_RotPwr;

  public SDRV_DrvManual(SwerveDriveSubsystem swerveDriveSubsystem, XboxController driverStick) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.driverStick = driverStick;
    addRequirements(this.swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Xe_r_LongPwr = -driverStick.getY(Hand.kLeft);
    Xe_r_LatPwr  =  driverStick.getX(Hand.kLeft);
    Xe_r_RotPwr  =  driverStick.getX(Hand.kRight);

    swerveDriveSubsystem.HolonomicDrv(Xe_r_LongPwr, Xe_r_LatPwr, Xe_r_RotPwr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.stopDrvMtrs();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }

}
