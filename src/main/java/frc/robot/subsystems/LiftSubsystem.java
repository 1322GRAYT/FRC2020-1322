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

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LiftSubsystem extends SubsystemBase {

  private CANSparkMax liftMotor;
  private DoubleSolenoid hangBar;
  /**
   * Creates a new LiftSubsystem.
   */
  public LiftSubsystem() {
    liftMotor = new CANSparkMax(Constants.ROBOT_LIFT, MotorType.kBrushless);
    hangBar = new DoubleSolenoid(Constants.HANG_BAR_0, Constants.HANG_BAR_1);
    enableBrake();
  }

  private void enableBrake() {
    liftMotor.setIdleMode(IdleMode.kBrake);
  }

  /**
   * Manipulate the hang bar
   * @param pos Position you wish the hang bar to be in
   */
  public void runLiftArm(Constants.SolenoidPosition pos) {
    if(pos.equals(Constants.SolenoidPosition.UP)) {
      hangBar.set(Value.kForward);
    } else if(pos.equals(Constants.SolenoidPosition.DOWN)) {
      hangBar.set(Value.kReverse);
    } else {
      hangBar.set(Value.kOff);
    }
  }
  
    /**
   * Runs the robot lift 
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
