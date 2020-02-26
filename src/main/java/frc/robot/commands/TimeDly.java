/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TimeDly extends CommandBase {
  /**
   * Command: TimeDly - Delays for a the amount
   * of time specified in the argument.
   */
  Timer  Xe_t_DlyTmr;
  double Xe_t_DlyPeriod;  

  public TimeDly(double Le_t_DlyPeriod) {
    Xe_t_DlyPeriod = Le_t_DlyPeriod;
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Xe_t_DlyTmr.reset();
    Xe_t_DlyTmr.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Xe_t_DlyTmr.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Xe_t_DlyTmr.get() > Xe_t_DlyPeriod);
  }
}
