/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ManualShootCommand extends CommandBase {
  BallSubsystem ballSubsystem;
  ShooterSubsystem shooterSubsystem;
  /**
   * Creates a new ManualShootCommand.
   */
  public ManualShootCommand(BallSubsystem bs, ShooterSubsystem ss) {
    addRequirements(bs, ss);
    ballSubsystem = bs;
    shooterSubsystem = ss;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread() {
      public void run() {
        Timer timer = new Timer();
        timer.reset();
        timer.start();
        // We are going to execute a bunch of stuff in order to
        // perfect shot and maximum.... na thats just BS this
        // is all just stuff that in my head makes sense to do
        
        // 1. Start spinning up the shooter motor, and reverse ball advance
        //         We're reversing the ball advance to ensure there is no ball
        //         currently in the shooter
        shooterSubsystem.runShooter(1);
        ballSubsystem.runAdvance(-.5);
        // Wait .2 Seconds
        while(timer.get() < .2) {}
        // 2. Stop ball advance
        ballSubsystem.runAdvance(0);
        // Reset Timer
        timer.reset(); // TODO: Make sure this doesn't stop the timer
        // Wait .7 Seconds for shooter to finish spinning up
        while(timer.get() < .7) {}
        // 3. Run ball advance forward to feed a ball to the shooter
        ballSubsystem.runAdvance(1);
        // Reset Timer
        timer.reset();
        // 4. Wait a while to stop shooting
        // TODO: Detect when there are no more balls
        while(timer.get() < 2)
        ballSubsystem.runAdvance(0);
        shooterSubsystem.runShooter(0);
      }
    }.run();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
