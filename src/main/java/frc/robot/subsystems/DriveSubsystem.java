/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  TalonFX drvFrontRight, drvFrontLeft, drvRearLeft, drvRearRight;
  CANSparkMax rotFrontRight, rotFrontLeft, rotRearLeft, rotRearRight;

  public DriveSubsystem() {
    drvFrontLeft = new TalonFX(Constants.SWRV_FR_LT_DRV);
    drvRearLeft = new TalonFX(Constants.SWRV_RR_LT_DRV);
    drvRearLeft.follow(drvFrontLeft);

    drvFrontRight = new TalonFX(Constants.SWRV_FR_RT_DRV);
    drvRearRight = new TalonFX(Constants.SWRV_RR_RT_DRV);
    drvRearRight.follow(drvFrontRight);


    rotFrontRight = new CANSparkMax(Constants.SWRV_FR_RT_ROT, MotorType.kBrushless);   
    rotFrontLeft  = new CANSparkMax(Constants.SWRV_FR_LT_ROT, MotorType.kBrushless);
    rotRearLeft   = new CANSparkMax(Constants.SWRV_RR_LT_ROT, MotorType.kBrushless);
    rotRearRight  = new CANSparkMax(Constants.SWRV_RR_RT_ROT, MotorType.kBrushless);
  }

  public void breakInDrive(double rotPower, double fwdPower) {
    this.drvFrontLeft.set(ControlMode.PercentOutput, fwdPower);
    this.drvFrontRight.set(ControlMode.PercentOutput, fwdPower);

    rotRearLeft.set(rotPower);
    rotFrontLeft.set(rotPower);
    rotRearRight.set(rotPower);
    rotFrontRight.set(rotPower);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
