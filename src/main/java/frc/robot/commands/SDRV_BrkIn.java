/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class SDRV_BrkIn extends CommandBase {
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private XboxController auxStick;
  /**
   * Creates a new BreakInDrive.
   */
  public SDRV_BrkIn(SwerveDriveSubsystem swerveDriveSubsystem, XboxController auxStick) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.auxStick = auxStick;
    addRequirements(this.swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveSubsystem.cntrlBrkInSwrvDrv(auxStick.getY(Hand.kLeft), auxStick.getY(Hand.kRight));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
