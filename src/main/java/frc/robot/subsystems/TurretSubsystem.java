/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private CANSparkMax shooter1, shooter2;
  CANPIDController PIDShooter1, PIDShooter2;
  CANEncoder ShootEncoder1, ShootEncoder2;

  double[] Shooter1kPIDF = {0.0001, 0, 0.0000, .000185};
  double[] Shooter2kPIDF = {0.0001, 0, 0.0000, .000185};

  /**
   * Creates a new ShooterPositionSubsystem.
   */
  public TurretSubsystem() {
    shooter1 = new CANSparkMax(Constants.SHOOTER_ONE, MotorType.kBrushless);
    shooter2 = new CANSparkMax(Constants.SHOOTER_TWO, MotorType.kBrushless);
    //shooter2.follow(shooter1, true);

    PIDShooter1 = shooter1.getPIDController();
    PIDShooter2 = shooter2.getPIDController();

    ShootEncoder1 = shooter1.getEncoder();
    ShootEncoder2 = shooter2.getEncoder();

    PIDShooter1.setP(Shooter1kPIDF[0]);
    PIDShooter1.setI(Shooter1kPIDF[1]);
    PIDShooter1.setD(Shooter1kPIDF[2]);
    PIDShooter1.setFF(Shooter1kPIDF[3]);

    PIDShooter2.setP(Shooter2kPIDF[0]);
    PIDShooter2.setI(Shooter2kPIDF[1]);
    PIDShooter2.setD(Shooter2kPIDF[2]);
    PIDShooter2.setFF(Shooter2kPIDF[3]);
  }

  int setBottomShooterSpeed = 5000;
  int setTopShooterSpeed = -2000;
  int tol = 300;
  public void pidShoot(boolean activate){
    PIDShooter1.setReference(activate ? setBottomShooterSpeed : 0, ControlType.kVelocity);
    PIDShooter2.setReference(activate ? setTopShooterSpeed : 0, ControlType.kVelocity);
  }

  public double getSpeed(){
    return ShootEncoder1.getVelocity();
  }

  public boolean isShooterAtSpeed(){
    return Math.abs(getSpeed() - setBottomShooterSpeed) < tol;
  }

  public void runShooter(double speed) {
    shooter1.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
