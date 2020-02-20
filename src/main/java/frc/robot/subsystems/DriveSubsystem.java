/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveShiftPos;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class DriveSubsystem extends SubsystemBase {
  TalonFX leftDriveRear, leftDriveFront, rightDriveRear, rightDriveFront;
  CANSparkMax leftRotRear, leftRotFront, rightRotRear, rightRotFront;
  DoubleSolenoid shifterSolenoid;

  public DriveSubsystem() {
    leftDriveFront = new TalonFX(Constants.FRONT_LEFT_DRIVE_DRIVE);
    leftDriveRear = new TalonFX(Constants.REAR_LEFT_DRIVE_DRIVE);
    leftDriveRear.follow(leftDriveFront);

    rightDriveFront = new TalonFX(Constants.FRONT_RIGHT_DRIVE_DRIVE);
    rightDriveRear = new TalonFX(Constants.REAR_RIGHT_DRIVE_DRIVE);
    rightDriveRear.follow(rightDriveFront);

    
    leftRotRear = new CANSparkMax(Constants.REAR_LEFT_DRIVE_ROTATE, MotorType.kBrushless);
    leftRotFront = new CANSparkMax(Constants.FRONT_LEFT_DRIVE_ROTATE, MotorType.kBrushless);
    rightRotRear = new CANSparkMax(Constants.REAR_RIGHT_DRIVE_ROTATE, MotorType.kBrushless);
    rightRotFront = new CANSparkMax(Constants.FRONT_RIGHT_DRIVE_ROTATE, MotorType.kBrushless);

    shifterSolenoid = new DoubleSolenoid(Constants.DRIVE_SHIFT_0, Constants.DRIVE_SHIFT_1);
  }

  public void breakInDrive(double rotPower, double fwdPower) {
    this.leftDriveFront.set(ControlMode.PercentOutput, fwdPower);
    this.rightDriveFront.set(ControlMode.PercentOutput, fwdPower);

    leftRotRear.set(rotPower);
    leftRotFront.set(rotPower);
    rightRotRear.set(rotPower);
    rightRotFront.set(rotPower);
  }

  public void shiftShifter(Constants.DriveShiftPos pos) {
    if(pos == DriveShiftPos.HIGH_GEAR) {
      this.shifterSolenoid.set(Value.kForward);
    } else {
      this.shifterSolenoid.set(Value.kReverse);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
