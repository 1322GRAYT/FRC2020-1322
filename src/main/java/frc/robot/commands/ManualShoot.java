/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualShoot extends CommandBase {
  BallSubsystem ballSubsystem;
  TurretSubsystem turretSubsystem;
  private XboxController auxStick;
  Timer shooterShutOffTimer = new Timer();

  /**
   * Creates a new ManualShoot Command.
   */
  public ManualShoot(BallSubsystem bs, TurretSubsystem ts, XboxController auxStick) {
    addRequirements(bs, ts);
    ballSubsystem = bs;
    turretSubsystem = ts;
    this.auxStick = auxStick;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Turn the shooter on to what we want it to be set to 100% of the time (also known as 4k RPM)
    turretSubsystem.pidShoot(true);
    shooterShutOffTimer.reset();
    shooterShutOffTimer.stop();
  }

  @Override
  public void execute() {
    // When shooter is at speed, shoot the balls, otherwise dont.
    if(turretSubsystem.isShooterAtSpeed()){
      ballSubsystem.runAdvance(0.85);
      ballSubsystem.runIntake(0.85);
      shooterShutOffTimer.start();
    }
    else{
      ballSubsystem.runAdvance(0);
      ballSubsystem.runIntake(0);
    }

    if ((auxStick.getXButton() == true) || ballSubsystem.getBallSensorIntake() || ballSubsystem.getBallSensorOutput()) {
      shooterShutOffTimer.reset();
    }

    SmartDashboard.putNumber("Shooter Speed", turretSubsystem.getSpeed());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean condMet = (auxStick.getXButtonReleased() || (shooterShutOffTimer.get() >= 2.5));
    return(condMet);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ballSubsystem.runAdvance(0);
    ballSubsystem.runIntake(0);
    turretSubsystem.pidShoot(false);
  }
}
