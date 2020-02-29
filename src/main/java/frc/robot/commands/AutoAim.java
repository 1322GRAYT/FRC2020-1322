/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAim extends CommandBase {
  private VisionSubsystem visionSubsystem;
  private AimSubsystem ballTurret;
  double XCorrect = 0, YCorrect = -2;

  /**
   * Creates a new AutoAim.
   */
  public AutoAim(AimSubsystem ballTurret, VisionSubsystem visionSubsystem, XboxController auxStick) {
    addRequirements(ballTurret);
    this.ballTurret = ballTurret;
    this.visionSubsystem = visionSubsystem;
    SmartDashboard.putBoolean("Auto Control", false);
    SmartDashboard.putNumber("Aim X Correct", XCorrect);
    SmartDashboard.putNumber("Aim Y Correct", YCorrect);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Notify the driver that the robot is taking over
    SmartDashboard.putBoolean("Auto Control", true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  boolean targetHeadingLeft = false;
  final double TOLERANCE = .5; // This is the in range value that allows it to stop. May try to tune lower when drives are available
  final double kP_PAN = -0.2; // -0.2 is the best so far
  final double kP_TILT = -0.2; // -0.2 is the best so far, may need to get more aggressive.
  final double maxTimeoutTimer = 1.0;
  Timer timeoutTimer = new Timer();
  @Override
  public void execute() {
    XCorrect = SmartDashboard.getNumber("Aim X Correct", XCorrect);
    YCorrect = SmartDashboard.getNumber("Aim Y Correct", YCorrect);
    var xError = visionSubsystem.getXOffset() + XCorrect;
    var yError = visionSubsystem.getYOffset() + YCorrect;
    var targetSeen = visionSubsystem.hasTarget();

    // Simple P controller will suffice here, if you want to get fancy, go for it!
    if((Math.abs(xError) > TOLERANCE) && targetSeen){
      ballTurret.pan(xError * kP_PAN);
    } else {
      ballTurret.tilt(0);
    }

    // Simple P controller will suffice here, if you want to get fancy, go for it!
    if((Math.abs(yError) > TOLERANCE) && targetSeen){
      ballTurret.tilt(yError * kP_TILT);
    } else {
      ballTurret.tilt(0);
    }

    // I want to give the camera time to see if it can find the target again
    if (!targetSeen){
      timeoutTimer.start();
    } else {
      timeoutTimer.stop();
      timeoutTimer.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("Auto Control", false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeoutTimer.get() > maxTimeoutTimer;
  }
}
