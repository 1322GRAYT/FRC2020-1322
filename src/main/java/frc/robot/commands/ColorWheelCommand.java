/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ColorWheelSubsystem;
import frc.robot.Constants.ColorWheelCommandExecute;;

public class ColorWheelCommand extends InstantCommand {

  ColorWheelSubsystem cws;
  ColorWheelCommandExecute cwce;
  /**
   * Creates a new ColorWheelCommand.
   */
  public ColorWheelCommand(ColorWheelSubsystem cws, ColorWheelCommandExecute cwce) {
    addRequirements(cws);
    this.cws = cws;
    this.cwce = cwce;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Gain Position Control
    if(cwce == ColorWheelCommandExecute.GAIN_POS_CONTROL){
      cws.cancelSpinnerIfRunning();
      cws.gainPositionControl();
    } else { // Gain Rotation Control
      cws.cancelSpinnerIfRunning();
      cws.gainRotationControl();
    }
  }
}
