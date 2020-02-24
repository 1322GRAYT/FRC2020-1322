/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDriveSubsystem;

public class BreakInDrive extends CommandBase {
  private SwerveDriveSubsystem swds;
  private XboxController auxStick;
  /**
   * Creates a new BreakInDrive.
   */
  public BreakInDrive(SwerveDriveSubsystem swds, XboxController auxStick) {
    this.swds = swds;
    this.auxStick = auxStick;
    addRequirements(this.swds);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swds.cntrlBrkInSwrvDrv(auxStick.getY(Hand.kLeft), auxStick.getY(Hand.kRight));
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
