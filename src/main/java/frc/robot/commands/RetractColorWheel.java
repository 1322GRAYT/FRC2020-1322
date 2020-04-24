/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SolenoidPosition;
import frc.robot.subsystems.ColorWheelSubsystem;

public class RetractColorWheel extends InstantCommand {
  private ColorWheelSubsystem cws;
  /**
   * Creates a new RetractColorWheel.
   */
  public RetractColorWheel(ColorWheelSubsystem cws) {
    addRequirements(cws);
    this.cws = cws;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cws.setWheelExtension(SolenoidPosition.DOWN);
  }
}
