/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;

public class BallSuckCommand extends CommandBase {
  private BallSubsystem ballSubsystem;
  private double speed = 0;
  /**
   * Creates a new BallSuckCommand.
   * @param bs BallSubsystem from RobotContainer class
   * @param speed Speed to run shooter at
   */
  public BallSuckCommand(BallSubsystem bs, double speed) {
    addRequirements(bs);
    ballSubsystem = bs;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ballSubsystem.runAdvance(speed);
    ballSubsystem.runIntake(speed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
