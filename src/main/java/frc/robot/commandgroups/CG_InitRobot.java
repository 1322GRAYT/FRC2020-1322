/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import frc.robot.commands.*;
import frc.robot.subsystems.DriveSubsystem;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CG_InitRobot extends SequentialCommandGroup {
  /**
   * Creates a new SDRV_RotInitRobot_CG.
   */
  public CG_InitRobot(DriveSubsystem driveSubsystem) {
    super(
      (new DRV_DrvRstGyro(driveSubsystem)),
      (new DRV_DrvRstEncdr(driveSubsystem))
    );

  }
}
