/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

  public CANSparkMax liftMotor;
  /**
   * Creates a new LiftSubsystem.
   */
  public LiftSubsystem() {
    liftMotor = new CANSparkMax(Constants.ROBOT_LIFT, MotorType.kBrushless);
    enableBrake();
  }

  private void enableBrake() {
    liftMotor.setIdleMode(IdleMode.kBrake);
  }
  
    /**
   * Runs the robot
   * @param speed Speed/Power you want to run at (-1 <- 0 -> 1)
   */
  public void runLift(double speed) {
    liftMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
