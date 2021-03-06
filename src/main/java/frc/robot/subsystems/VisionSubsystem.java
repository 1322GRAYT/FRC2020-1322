/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {

  private boolean validTarget = false;
  private double xOffset, yOffset, rotOffset;

  /**
   * Creates a new VisionSubsystem.
   */
  public VisionSubsystem() {

  }

  public double getXOffset() {
    return xOffset;
  }

  public double getYOffset() {
    return yOffset;
  }

  public double getRotOffset() {
    return rotOffset;
  }

  public boolean hasTarget() {
    return validTarget;
  }

  /*
  int horz_FOV = 320/2;
  int 
  double[] targetAngles(){
    double nx = (1/160)*(llTable.getEntry("tx").getDouble(0) - 159.5);
    double ny = (1/120)*(119.5 - llTable.getEntry("tx").getDouble(0));


    
  }*/

  public double getXError(){
    return (xOffset - 159.5);
  }

  public double getYError(){
    return (119.5 - yOffset);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    NetworkTable llTable = NetworkTableInstance.getDefault().getTable("limelight");
    xOffset = llTable.getEntry("tx").getDouble(0);
    yOffset = llTable.getEntry("ty").getDouble(0);
    rotOffset = llTable.getEntry("ts").getDouble(0);
    validTarget = !(llTable.getEntry("tv").getDouble(0) == 0); // This NetworkTable Value returns 1 if there is a valid target
  }
}
