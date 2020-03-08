/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commandgroups;

import frc.robot.commands.*;
import frc.robot.subsystems.BallSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class CG_AutoShootDrvBack3 extends SequentialCommandGroup {
  /**
   * Creates a new SDRV_RotInitRobot_CG.
   */
  public CG_AutoShootDrvBack3(DriveSubsystem  driveSubsystem,
                              BallSubsystem   ballSubsystem,
                              TurretSubsystem turretSubsystem,
                              XboxController  auxStick) {
    super(
      (new DRV_DrvRstGyro(driveSubsystem)),
      (new DRV_DrvRstEncdr(driveSubsystem)),
      (new TimeDly(0.25)),
      (new ManualShoot(ballSubsystem, turretSubsystem, auxStick)),
      (new TimeDly(0.50)),
      (new DRV_DrvFwdFeet(driveSubsystem, -3.0, 0.0))
    );

  }
}
