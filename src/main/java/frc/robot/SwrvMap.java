/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * SwrvMap: Identifies the location of the Swerve Drive Caddies, i.e. maps each
 * caddy to an integer value for array indexing.
 */
public class SwrvMap {
  public static final int RtFt = 0; // Swerve Drive Caddy: Right Front
  public static final int LtFt = 1; // Swerve Drive Caddy: Left  Front
  public static final int LtRr = 2; // Swerve Drive Caddy: Left  Rear
  public static final int RtRr = 3; // Swerve Drive Caddy: Right Rear
  public static final int NumOfCaddies = 4; //Total Number of Swerve Drive Caddies
}
