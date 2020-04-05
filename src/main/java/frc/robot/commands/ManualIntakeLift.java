/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.calibrations.K_BALL;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.BallSubsystem.IntakeLiftPosition;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class ManualIntakeLift extends CommandBase {
  private BallSubsystem ballSubsystem;
  private XboxController auxStick;

  /**
   * Creates a new ManualIntakeLift.
   */
  public ManualIntakeLift(BallSubsystem ballSubsystem, XboxController auxStick) {
    this.ballSubsystem = ballSubsystem;
    this.auxStick = auxStick;
    addRequirements(this.ballSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean intakeBallHold = (ballSubsystem.getBallSensorIntake() && ballSubsystem.getBallSensorOutput());

    double powerIntake = auxStick.getY(Hand.kRight);
    if((powerIntake > .1) && !intakeBallHold) {  
      ballSubsystem.runIntake(K_BALL.KeBAL_r_NormPwrIntakeLoad);  /* Intake Ball at Constant Speed */
    } else if (powerIntake < -.1) {
      ballSubsystem.runIntake(powerIntake);  /* Clear Ball at Variable Speed */
    } else {
      ballSubsystem.runIntake(0);
    }
    double powerIntakeLift = auxStick.getTriggerAxis(Hand.kRight) - auxStick.getTriggerAxis(Hand.kLeft);
    if(Math.abs(powerIntakeLift) > .1) {
      ballSubsystem.runLift(powerIntakeLift);
    } else {
      ballSubsystem.runLift(0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.runIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
