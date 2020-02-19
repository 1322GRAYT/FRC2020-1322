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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private TalonSRX pan, tilt;
  private CANSparkMax shooter1, shooter2;

  /**
   * Creates a new ShooterPositionSubsystem.
   */
  public TurretSubsystem() {
    pan = new TalonSRX(Constants.SHOOTER_AIM_PAN);
    tilt = new TalonSRX(Constants.SHOOTER_AIM_TILT);
    shooter1 = new CANSparkMax(Constants.SHOOTER_ONE, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.SHOOTER_TWO, MotorType.kBrushless);
    shooter2.follow(shooter1, true);

    int CruiseVelocity = 250; // Ticks per 100ms, for pan and tilt
    int Acceleration = 30; // Ticks per 100ms per second, for pan and tilt
    
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

  // TODO: Calculate Velocity for different target sizes
  // Smaller Target = Futher Away = More Velocity Needed
  public void prepareShooter(double targetSize) {
    shooter1.set(1);
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

  public void manualControl(double pan, double tilt){
    this.pan.set(ControlMode.PercentOutput, pan);
    this.tilt.set(ControlMode.PercentOutput, tilt);
  }

  public void panGoToAngle(double angle){
    this.pan.set(ControlMode.MotionMagic, angle);
  }

  public double panEncoder(){
    return PanTickToDegrees(pan.getSelectedSensorPosition());
  }

  public double tiltEncoder(){
    return TiltTickToDegrees(tilt.getSelectedSensorPosition());
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
  }
}
