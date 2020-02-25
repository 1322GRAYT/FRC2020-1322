/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AimSubsystem extends SubsystemBase {
  private TalonSRX pan, tilt;

  private int panEncTicks, tiltEncTicks;

  int CruiseVelocity = 250; // Ticks per 100ms, for pan and tilt
  int Acceleration = 30; // Ticks per 100ms per second, for pan and tilt
  /**
   * Creates a new AimSubsystem.
   */
  public AimSubsystem() {

    pan = new TalonSRX(Constants.SHOOTER_AIM_PAN);
    tilt = new TalonSRX(Constants.SHOOTER_AIM_TILT);
    double[] PanTiltkPIDF = {0.1, 0, 0, 4.5};
    

    /* Config Pan used for Primary PID [Velocity] */
    pan.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    pan.configSetParameter(ParamEnum.eProfileParamSlot_P, PanTiltkPIDF[0], 0, 0);
    pan.configSetParameter(ParamEnum.eProfileParamSlot_I, PanTiltkPIDF[1], 0, 0);
    pan.configSetParameter(ParamEnum.eProfileParamSlot_D, PanTiltkPIDF[2], 0, 0);
    pan.configSetParameter(ParamEnum.eProfileParamSlot_F, PanTiltkPIDF[3], 0, 0);
    pan.configMotionCruiseVelocity(CruiseVelocity);
    pan.configMotionAcceleration(Acceleration);

    /* Config Pan used for Primary PID [Velocity] */
    tilt.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    tilt.configSetParameter(ParamEnum.eProfileParamSlot_P, PanTiltkPIDF[0], 0, 0);
    tilt.configSetParameter(ParamEnum.eProfileParamSlot_I, PanTiltkPIDF[1], 0, 0);
    tilt.configSetParameter(ParamEnum.eProfileParamSlot_D, PanTiltkPIDF[2], 0, 0);
    tilt.configSetParameter(ParamEnum.eProfileParamSlot_F, PanTiltkPIDF[3], 0, 0);
    tilt.configMotionCruiseVelocity(CruiseVelocity);
    tilt.configMotionAcceleration(Acceleration);
  }

  public void manualControl(double pan, double tilt){
    this.pan.set(ControlMode.PercentOutput, pan);
    this.tilt.set(ControlMode.PercentOutput, tilt);
  }

  public void pan(double power){
    this.pan.set(ControlMode.PercentOutput, power);
  }

  public void tilt(double power){
    this.tilt.set(ControlMode.PercentOutput, power);
  }

  public void panGoToAngle(double angle){
    this.pan.set(ControlMode.MotionMagic, angle);
  }

  public double panEncoder(){
    return PanTickToDegrees(panEncTicks);
  }

  public double tiltEncoder(){
    return TiltTickToDegrees(tiltEncTicks);
  }

  int PanDegreeToTick(double in){
    return (int)(in / (9/7 * 16/187));
  }
  
  double PanTickToDegrees(int in){
    return in * 9/(7*4) * 16/187;
  }

  double TiltTickToDegrees(int in){
    return (double)in / 121.8;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // Update Encoder Ticks
    panEncTicks = pan.getSelectedSensorPosition();
    tiltEncTicks = tilt.getSelectedSensorPosition();
  }
}
