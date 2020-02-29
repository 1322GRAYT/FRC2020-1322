/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveShiftPos;

public class ShiftSubsystem extends SubsystemBase {
  private DoubleSolenoid shifterSolenoid;
  private DriveShiftPos  selectedGear;


  public ShiftSubsystem() {
    shifterSolenoid = new DoubleSolenoid(Constants.DRIVE_SHIFT_0, Constants.DRIVE_SHIFT_1);
  }

  public void shiftShifter(Constants.DriveShiftPos pos) {
    selectedGear = pos;
    if(selectedGear == DriveShiftPos.HIGH_GEAR) {
      this.shifterSolenoid.set(Value.kForward);
    } else {
      this.shifterSolenoid.set(Value.kReverse);
    }
  }


  public DriveShiftPos getSelectedGear() {
    return (selectedGear);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
