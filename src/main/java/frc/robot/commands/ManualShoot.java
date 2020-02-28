/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualShoot extends CommandBase {
  BallSubsystem ballSubsystem;
  TurretSubsystem shooterSubsystem;
  private XboxController auxStick;

  /**
   * Creates a new ManualShoot Command.
   */
  public ManualShoot(BallSubsystem bs, TurretSubsystem ss, XboxController auxStick) {
    addRequirements(bs, ss);
    ballSubsystem = bs;
    shooterSubsystem = ss;
    this.auxStick = auxStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Turn the shooter on to what we want it to be set to 100% of the time (also known as 4k RPM)
    shooterSubsystem.pidShoot(true);
  }

  @Override
  public void execute() {
    // When shooter is at speed, shoot the balls, otherwise dont.
    if(shooterSubsystem.isShooterAtSpeed()){
      ballSubsystem.runAdvance(1);
    }
    else{
      ballSubsystem.runAdvance(0);
    }
    SmartDashboard.putNumber("Shooter Speed", shooterSubsystem.getSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return auxStick.getYButtonReleased();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.runAdvance(0);
    shooterSubsystem.pidShoot(false);
  }
}
