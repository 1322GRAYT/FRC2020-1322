/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.DriveShiftPos;
import frc.robot.subsystems.ShiftSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class ShiftCommand extends InstantCommand {
  private ShiftSubsystem ss;
  private DriveShiftPos shiftPos;
  public ShiftCommand(ShiftSubsystem ss, DriveShiftPos pos) {
    this.ss = ss;
    shiftPos = pos;
    addRequirements(this.ss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.ss.shiftShifter(shiftPos);
  }
}
