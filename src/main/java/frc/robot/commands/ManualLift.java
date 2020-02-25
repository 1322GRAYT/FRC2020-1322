/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.LiftSubsystem;

public class ManualLift extends CommandBase {
  private LiftSubsystem liftSubsystem;
  private XboxController auxStick;
  private double speed;

  /**
   * Creates a new ManualLift.
   */
  public ManualLift(LiftSubsystem liftSubsystem, XboxController auxStick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.liftSubsystem = liftSubsystem;
    addRequirements(this.liftSubsystem);

    this.auxStick = auxStick;
  }
  
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Right Trigger to go forward, Select button to go backwards
    double liftPower = (auxStick.getRawAxis(3)) - (auxStick.getRawButton(7)?1:0);
    liftSubsystem.runLift(liftPower);

    int dPadPos = auxStick.getPOV();
    if ((dPadPos > 350 || dPadPos < 10) && dPadPos != -1){ // D-Pad Up
      liftSubsystem.runLiftArm(Constants.SolenoidPosition.UP);
    } else if (auxStick.getPOV() > 170 && auxStick.getPOV() < 200){ // D-Pad Down
      liftSubsystem.runLiftArm(Constants.SolenoidPosition.DOWN);
    }

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
