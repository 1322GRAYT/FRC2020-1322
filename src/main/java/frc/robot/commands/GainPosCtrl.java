/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorWheelColor;
import frc.robot.Constants.SolenoidPosition;
import frc.robot.subsystems.ColorWheelSubsystem;

public class GainPosCtrl extends CommandBase {
  ColorWheelSubsystem cws;
  private int numTimesPassedFieldColor;
  private ColorWheelColor lastColor;
  /**
   * Creates a new GainPosCtrl.
   */
  public GainPosCtrl(ColorWheelSubsystem cws) {
    addRequirements(cws);
    this.cws = cws;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    cws.setWheelExtension(SolenoidPosition.UP);
    cws.setWheelSpeed(Constants.COLOR_WHEEL_SPIN_SPEED * .5);
    numTimesPassedFieldColor = 0;
    lastColor = cws.getCurrentColorValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(lastColor != cws.getCurrentColorValue()) {
      lastColor = cws.getCurrentColorValue();
      if(cws.getFieldColorFromRobotColor(cws.getCurrentColorValue()) == cws.getGameDataColor()) {
        numTimesPassedFieldColor++;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    cws.stopSpinnerAndLower();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numTimesPassedFieldColor > 1;
  }
}
