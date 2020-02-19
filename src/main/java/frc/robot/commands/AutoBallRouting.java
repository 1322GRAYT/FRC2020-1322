/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class AutoBallRouting extends CommandBase {
  /**
   * Creates a new BallIntake.
   */

  BallSubsystem ballSubsystem;
  public AutoBallRouting(BallSubsystem ballSubsystem) {
    this.ballSubsystem = ballSubsystem;
    addRequirements(this.ballSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballSubsystem.runAdvance(0.35);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.runAdvance(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !ballSubsystem.getBallSensorIntake();
  }
}
