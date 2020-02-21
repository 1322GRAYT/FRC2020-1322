/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax shooter1, shooter2;

  /**
   * Creates a new ShooterPositionSubsystem.
   */
  public TurretSubsystem() {
    shooter1 = new CANSparkMax(Constants.SHOOTER_ONE, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.SHOOTER_TWO, MotorType.kBrushless);
    shooter2.follow(shooter1, true);
  }

  // TODO: Calculate Velocity for different target sizes
  // Smaller Target = Futher Away = More Velocity Needed
  public void prepareShooter(double targetSize) {
    shooter1.set(1);
  }

  public void runShooter(double speed) {
    shooter1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
