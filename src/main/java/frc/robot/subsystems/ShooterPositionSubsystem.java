/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterPositionSubsystem extends SubsystemBase {

  private TalonSRX pan, tilt;

  /**
   * Creates a new ShooterPositionSubsystem.
   */
  public ShooterPositionSubsystem() {
    //pan = new TalonSRX(Constants.SHOOTER_AIM_PAN);
    //tilt = new TalonSRX(Constants.SHOOTER_AIM_TILT);
    // For Limelight Test Bot
    pan = new TalonSRX(9);
    tilt = new TalonSRX(10);
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