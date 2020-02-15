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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class BallSubsystem extends SubsystemBase {

  public enum IntakeLiftPosition {UP, DOWN};

  private boolean intakeSensorStatus = false, outputSensorStatus = false;

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
     * Raises or Lowers the ball intake by running it for 1 second.
     * There are physical limits, and physical limit
     * switches so we don't risk breaking anything
     * @param pos Lift Position to go to
     */
    public void raiseLowerIntake(IntakeLiftPosition pos) {
      new Thread() {
        public void run() {
          Timer liftTimer = new Timer();
          liftTimer.reset();
          liftTimer.start();
          runLift(((pos == IntakeLiftPosition.UP) ? 1 : -1) * Constants.RAISE_LOWER_INTAKE_SPEED);
          while(liftTimer.get() < Constants.LOWER_INTAKE_TIME_SEC) {
            // Do Nothing
          }
          runLift(0);
          liftTimer.stop();
        }
      }.run();
    }

    /**
     * Runs the intake lift
     * This is private because the method users should be using is
     * called Raise/Lower Lift
     * @param speed Speed/Power you want to run at (-1 <- 0 -> 1)
     */
    private void runLift(double speed) {
       ballIntakeLift.set(ControlMode.PercentOutput, speed);
    }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.intakeSensorStatus = ballSenseIntake.get();
    this.outputSensorStatus = ballSenseOutput.get();
  }
}
