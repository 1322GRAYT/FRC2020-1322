/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualShootCommand extends InstantCommand {
  BallSubsystem ballSubsystem;
  TurretSubsystem shooterSubsystem;
  /**
   * Creates a new ManualShootCommand.
   */
  public ManualShootCommand(BallSubsystem bs, TurretSubsystem ss) {
    addRequirements(bs, ss);
    ballSubsystem = bs;
    shooterSubsystem = ss;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread() {
      public void run() {
        System.out.println("Start thinng");
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
        while(timer.get() < 1) {}
        // 3. Run ball advance forward to feed a ball to the shooter
        ballSubsystem.runAdvance(1);
        // Reset Timer
        timer.reset();
        // 4. Wait till balls are empty to stop shooting
        while(true) {
          // When we see a ball, reset the timer
          if(ballSubsystem.getBallSensorOuput()) {
            timer.reset();
          }
          // If we haven't seen a ball for over 1.5 seconds, stop shooting
          if(timer.get() > 1.5) {
            break;
          }
        }
        // Stop Motors
        ballSubsystem.runAdvance(0);
        shooterSubsystem.runShooter(0);
      }
    }.run();
  }
}
