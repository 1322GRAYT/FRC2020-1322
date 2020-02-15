/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class TestShooterPos extends CommandBase {
  ShooterSubsystem ss;
  double speed;
  /**
   * Creates a new TestShooterPos.
   */
  public TestShooterPos(ShooterSubsystem ss, double speed) {
    addRequirements(ss);
    this.ss = ss;
    this.speed = speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ss.pan(speed);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
