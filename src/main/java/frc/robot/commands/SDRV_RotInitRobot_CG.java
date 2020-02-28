/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import frc.robot.subsystems.SwerveDriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class SDRV_RotInitRobot_CG extends SequentialCommandGroup {
  /**
   * Creates a new SDRV_RotInitRobot_CG.
   */
  public SDRV_RotInitRobot_CG(SwerveDriveSubsystem swerveDriveSubsystem) {
    super(
      (new SDRV_RotRstZero(swerveDriveSubsystem)),
      (new SDRV_RotFindZero(swerveDriveSubsystem))
    );

  }
}
