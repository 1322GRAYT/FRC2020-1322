/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.ColorWheelColor;
import frc.robot.Constants.SolenoidPosition;
import frc.robot.subsystems.ColorWheelSubsystem;

public class GainRotCtrl extends CommandBase {
  ColorWheelSubsystem cws;
  boolean stopAtEnd;
  ColorWheelColor startColor, lastColor = ColorWheelColor.UNKNOWN;
  int numberOfTimesSeenStartColor;
  /**
   * Creates a new GainRotCtrl.
   */
  public GainRotCtrl(ColorWheelSubsystem cws, boolean stopAtEnd) {
    addRequirements(cws);
    this.cws = cws;
    this.stopAtEnd = stopAtEnd;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    numberOfTimesSeenStartColor = 0;
    cws.setWheelExtension(SolenoidPosition.UP);
    cws.setWheelSpeed(Constants.COLOR_WHEEL_SPIN_SPEED);
    startColor = cws.getCurrentColorValue();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cws.getCurrentColorValue() != lastColor) {
      //New Color Detected
      if(cws.getCurrentColorValue() == startColor) {
        // We see the start color, lets add it
        numberOfTimesSeenStartColor++;
      }
      // Update last color with the current color
      lastColor = cws.getCurrentColorValue();
      // SmartDashboard Update
      SmartDashboard.putNumber("NumSeenStartColor", numberOfTimesSeenStartColor);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(stopAtEnd) {
      cws.setWheelExtension(SolenoidPosition.DOWN);
      cws.setWheelSpeed(0);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return numberOfTimesSeenStartColor > 6;
  }
}
