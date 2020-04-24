/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.CommandBase;

public class DRV_DrvRstGyro extends CommandBase {
  /**
   * Command: DRV_DrvRstGyro  
   */
  private DriveSubsystem driveSubsystem;
    int Le_Cnt_Dly;

  public DRV_DrvRstGyro(DriveSubsystem driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    Le_Cnt_Dly = 0;
    addRequirements(this.driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  Le_Cnt_Dly += 1; 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return (Le_Cnt_Dly >= 2);
  }
}
