/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.ShooterPositionSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoAimCommand extends CommandBase {
  /**
   * Creates a new AutoAimCommand.
   */

  private VisionSubsystem visonSub;
  private ShooterPositionSubsystem aimSub;

  public AutoAimCommand(ShooterPositionSubsystem aimSub, VisionSubsystem visionSub) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(aimSub, visionSub);
    this.visonSub = visionSub;
    this.aimSub = aimSub;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    new Thread() {
      private double aimSpeed = .1;
      private double goodEnoughX = .2; // If the offset is +/- this value, then we'll stop moving. Greater number = less accuracy;
      private double goodEnoughY = .2; // If the offset is +/- this value, then we'll stop moving. Greater number = less accuracy;
      public void run() {
        System.out.println("Running 1322AutoAim Beta 0.0.1");
        while(true) {
          /* 
          TODO: Use a PID Loop Here. I am dumb and don't know how to do it properly yet
          so I'm going to just do a primitive style of control for aiming. (Just move it
          very slowly, always.)
           */
          // No Target, don't bother positioning
          if(!visonSub.hasTarget()) break; 
  
          // Calculate if we need to do positioning
          boolean needToPositionPan = Math.abs(visonSub.getXOffset()) > goodEnoughX;
          boolean needToPositionTilt = Math.abs(visonSub.getYOffset()) > goodEnoughY;
  
          // Lets do L/R Pos
          if(needToPositionPan) {
            // If the offset is negative
            boolean isThereNegativeXOffset = visonSub.getXOffset() < 0;
            // If the offset is negative, we multiply the speed by -1 to go in reverse
            aimSub.pan((isThereNegativeXOffset ? -1 : 1) * aimSpeed);
          } else {
            aimSub.pan(0);
          }
  
          // Lets do Up/Down Pos
          if(needToPositionTilt) {
             // If the offset is negative
            boolean isThereNegativeYOffset = visonSub.getXOffset() < 0;
            // If the offset is negative, we multiply the speed by -1 to go in reverse
            aimSub.tilt((isThereNegativeYOffset ? -1 : 1) * aimSpeed);
          } else {
            aimSub.tilt(0);
          }
          
          // If both offsets are 'good enough' we can break the loop
          if(!needToPositionPan && !needToPositionTilt) {
            aimSub.pan(0);
            aimSub.tilt(0);
            break;
          }
  
          // TODO: Add Check for a Manual Override: If So, Cancel Loop
        }
        System.out.println("Finished 1322AutoAim Beta 0.0.1");
      }
    }.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
