/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallSubsystem extends SubsystemBase {

  public enum IntakeLiftPosition {UP, DOWN};

  private IntakeLiftPosition currentPos = IntakeLiftPosition.DOWN;

  private boolean intakeSensorStatus = false, outputSensorStatus = false, runAdvanceAutonomously = false;

  private TalonSRX ballIntakeSuck, ballAdvance, ballIntakeLift;
  private DigitalInput ballSenseIntake, ballSenseOutput;
  /**
   * Creates a new BallSubsystem.
   */
  public BallSubsystem() {
    ballIntakeSuck = new TalonSRX(Constants.SHOOTER_BALL_INTAKE);
    ballAdvance = new TalonSRX(Constants.SHOOTER_BALL_ADVANCE);
    ballIntakeLift = new TalonSRX(Constants.SHOOTER_INTAKE_LIFT);
    ballSenseIntake = new DigitalInput(Constants.BALL_SENSE_INPUT);
    ballSenseOutput = new DigitalInput(Constants.BALL_SENSE_OUTPUT);
  }
  
  public boolean getBallSensorIntake() {
    return intakeSensorStatus;
  }

  public boolean getBallSensorOuput() {
    return outputSensorStatus;
  }

  public IntakeLiftPosition getCurrentIntakePosition() {
    return currentPos;
  }
  public void setCurrentIntakePosition(IntakeLiftPosition pos) {
    this.currentPos = pos;
  }
  
    /**
     * Runs the ball intake
     * @param speed Speed/Power you want to run at (-1 <- 0 -> 1)
     */
    public void runIntake(double speed) {
      ballIntakeSuck.set(ControlMode.PercentOutput, speed);
    }
    /**
     * Runs the ball advancer
     * @param speed Speed/Power you want to run at (-1 <- 0 -> 1)
     */
    public void runAdvance(double speed) {
      ballAdvance.set(ControlMode.PercentOutput, speed);
    }

    /**
     * Runs the intake lift
     * @param speed Speed/Power you want to run at (-1 <- 0 -> 1)
     */
    public void runLift(double speed) {
       ballIntakeLift.set(ControlMode.PercentOutput, speed);
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.intakeSensorStatus = ballSenseIntake.get();
    this.outputSensorStatus = ballSenseOutput.get();
    //System.out.println("Intake Sensor: " + intakeSensorStatus + " Output Sensor: " + outputSensorStatus);

    // If We have a ball at the intake, and none at the output, lets run the advance to move it
    if(this.intakeSensorStatus && !this.outputSensorStatus){
      runAdvance(.4);
      runAdvanceAutonomously = true;
    } 
    if(!this.intakeSensorStatus && runAdvanceAutonomously) {
      runAdvance(0);
      runAdvanceAutonomously = false;
    }

    // Update SmartDashboard
    SmartDashboard.putBoolean("Intake Status", this.intakeSensorStatus);
    SmartDashboard.putBoolean("Output Status", this.outputSensorStatus);
  }
}
