/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {

  private TalonSRX pan, tilt;
  private CANSparkMax shooter1, shooter2;

  /**
   * Creates a new ShooterPositionSubsystem.
   */
  public ShooterSubsystem() {
    pan = new TalonSRX(Constants.SHOOTER_AIM_PAN);
    tilt = new TalonSRX(Constants.SHOOTER_AIM_TILT);
    shooter1 = new CANSparkMax(Constants.SHOOTER_ONE, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.SHOOTER_TWO, MotorType.kBrushless);
    shooter2.follow(shooter1, true);
    // For Limelight Test Bot
    // pan = new TalonSRX(9);
    // tilt = new TalonSRX(10);
  }

  public void runShooter(double speed) {
    shooter1.set(speed);
  }
  
  public void pan(double speed) {
    pan.set(ControlMode.PercentOutput, speed);
  }

  public void tilt(double speed) {
    tilt.set(ControlMode.PercentOutput, speed);
  }

  public int getPanEnc() {
    return pan.getSelectedSensorPosition();
  }
  public int getTiltEnc() {
    return tilt.getSelectedSensorPosition();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
