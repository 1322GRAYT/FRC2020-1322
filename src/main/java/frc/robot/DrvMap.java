/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * SwrvMap: Identifies the Drive System Motors, i.e. maps each
 * Drive Motor to an integer value for array indexing.
 */
public class DrvMap {
  public static final int RtMstr = 0; // Drive Motor: Right Master
  public static final int RtSlv  = 1; // Drive Motor: Right Slave
  public static final int LtMstr = 2; // Drive Motor: Left Master
  public static final int LtSlv  = 3; // Drive Motor: Left Slave
  public static final int NumOfMtrs = 4; //Total Number of Motors

  public static final int RtAssy = 0; // Drive Assembly: Right Drive Motor Assembly
  public static final int LtAssy = 1; // Drive Assembly: Left  Drive Motor Assembly
  public static final int NumOfAssy = 2; //Total Number of Drive Motor Assemblies

}
