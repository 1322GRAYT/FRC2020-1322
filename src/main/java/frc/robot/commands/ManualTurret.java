/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.AimSubsystem;
import frc.robot.subsystems.TurretSubsystem;

public class ManualTurret extends CommandBase {
  private AimSubsystem ballTurret;
  private XboxController auxStick;

  /**
   * Creates a new TurretManualControl.
   */
  public ManualTurret(AimSubsystem ballTurret, XboxController auxStick) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.auxStick = auxStick;
    this.ballTurret = ballTurret;
    addRequirements(this.ballTurret);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ballTurret.manualControl(-this.auxStick.getX(Hand.kLeft), this.auxStick.getY(Hand.kLeft));
    SmartDashboard.putNumber("PanAngle", ballTurret.panEncoder());
    SmartDashboard.putNumber("TiltAngle", ballTurret.tiltEncoder());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
