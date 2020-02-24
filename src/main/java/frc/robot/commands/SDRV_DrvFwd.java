/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class SDRV_DrvFwd extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  SwerveDriveSubsystem swds;

  private double d1; 
  private double startAngle;
  private double speed;
  public static final double WHEEL_CIRCUMFERENCE = 4 * Math.PI;
  public static final double TOTAL_SENSOR_POS = 1024;
  public static final double DISTANCE = WHEEL_CIRCUMFERENCE / TOTAL_SENSOR_POS;

  public SDRV_DrvFwd(SwerveDriveSubsystem swds, double Le_l_DsrdDist, double speed) {
    this.swds = swds;

    d1 = DISTANCE * Le_l_DsrdDist * 12;
    this.speed = speed;
    addRequirements(this.swds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startAngle = swds.getNavX().getYaw();
    swds.resetDrvEncdrs();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swds.driveForwardToDist(d1, startAngle, 0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swds.calcDrvPosErr(d1)) < DISTANCE;
  }
}
