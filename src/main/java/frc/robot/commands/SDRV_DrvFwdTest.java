/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.subsystems.SwerveDriveModule.TeMtrDirctn;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SDRV_DrvFwdTest extends CommandBase {
  /**
   * Command: SDRV_DrvFwd Command to Drive the Swerve Drive
   * Forward or Backwards at a specific Power Request. 
   */
  private SwerveDriveSubsystem swerveDriveSubsystem;
  private int Xe_i_BnkIdx;
  private double Xe_r_DrvPwr;
  private TeMtrDirctn Xe_e_MtrDirctn;

  public SDRV_DrvFwdTest(SwerveDriveSubsystem swerveDriveSubsystem, int Xe_i_BnkIdx, double Xe_r_DrvPwr) {
    this.swerveDriveSubsystem = swerveDriveSubsystem;
    this.Xe_i_BnkIdx = Xe_i_BnkIdx;
    this.Xe_r_DrvPwr = Xe_r_DrvPwr;
    addRequirements(this.swerveDriveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Xe_e_MtrDirctn = TeMtrDirctn.Fwd;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerveDriveSubsystem.runSwerveCaddyAtPwr(Xe_i_BnkIdx, Xe_e_MtrDirctn, Xe_r_DrvPwr);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveDriveSubsystem.runSwerveCaddyAtPwr(Xe_i_BnkIdx, Xe_e_MtrDirctn, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (false);
  }
}
