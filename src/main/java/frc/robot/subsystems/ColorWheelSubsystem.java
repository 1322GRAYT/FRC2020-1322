/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ColorWheelColor;
import frc.robot.Constants.SolenoidPosition;


/**
 * NOTES ON THE COLOR WHEEL
 * 
 * Order of colors (Going Clockwise around the wheel)
 * Blue, Green, Red, Yellow, Blue, Green, Red, Yellow
 * 
 * The Colors that we are reading on the robot ARE NOT
 * the colors that the field is reading!!!
 * 
 * That Being said, Here's the matchup: 
 * (If we're reading 90 degrees off from the field sensor)
 * What We Read   | What The Field is reading  
 * Blue           |   Red
 * Green          |   Yellow
 * Red            |   Blue
 * Yellow         |   Green
 * (The Field Reads 2 Colors Ahead of What we're reading)
 */

public class ColorWheelSubsystem extends SubsystemBase {

  // New Color Sensor
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
  private final ColorMatch m_colorMatcher = new ColorMatch();
  private int currentProx;

  // New CANTalon Motor
  private VictorSPX colorWheelMotor;

  // Air Things
  private DoubleSolenoid wheelExtension;

  // Boolean to track if we have the Game Data
  private boolean haveGameData = false;

  // Store Game Data (if we have it)
  private ColorWheelColor gameDataColor = ColorWheelColor.UNKNOWN;

  // Current Color of sensor (Read once once per loop)
  private ColorWheelColor currentSensorColor = ColorWheelColor.UNKNOWN;

  // Threads
  private Thread rotThread;
  private Thread posThread;
  
  public ColorWheelSubsystem() {
    // Init Motor
    colorWheelMotor = new VictorSPX(Constants.COLOR_WHEEL_SPINNER);
    // Add Color Calibrations to Color matches
    m_colorMatcher.addColorMatch(Constants.kBlueTarget);
    m_colorMatcher.addColorMatch(Constants.kGreenTarget);
    m_colorMatcher.addColorMatch(Constants.kRedTarget);
    m_colorMatcher.addColorMatch(Constants.kYellowTarget);  
    // Init Solenoid
    wheelExtension = new DoubleSolenoid(Constants.COLOR_WHEEL_0, Constants.COLOR_WHEEL_1);
    colorWheelMotor.setNeutralMode(NeutralMode.Brake);
  }

  public void setWheelSpeed(double speed) {
    colorWheelMotor.set(ControlMode.PercentOutput, speed);
  }

  public int getProximity() {
    return currentProx;
  }

  public ColorWheelColor getCurrentColorValue() {
    return currentSensorColor;
  }

  public ColorWheelColor getGameDataColor() {
    return gameDataColor;
  }

  /**
   * The Color the robot is reading is not the color that the field is reading.
   * This method converts the color that the robot is reading to the color
   * that the field is reading.
   * 
   * As of right now, this method is setup if we read the color that is
   * 90 degrees off from the field's sensor. This can be changed based
   * off of what the drivers end up doing during practice.
   * 
   * @param color Color you want converted to field color
   * @return What color the field would be reading based off of the color given
   */
  public ColorWheelColor getFieldColorFromRobotColor(ColorWheelColor color) {
      switch(color) {
        case BLUE   : return ColorWheelColor.RED;
        case GREEN  : return ColorWheelColor.YELLOW;
        case RED    : return ColorWheelColor.BLUE;
        case YELLOW : return ColorWheelColor.GREEN;
        default     : return ColorWheelColor.UNKNOWN;
      }
  }

  /**
   * Get The Currently Detect Color at the Sensor
   * @return ColorWheelColor Detected Color
   */
  private ColorWheelColor getCurrentColor() {
    ColorMatchResult match = m_colorMatcher.matchClosestColor(m_colorSensor.getColor());
    if (match.color == Constants.kBlueTarget) {
      return ColorWheelColor.BLUE;
    } else if (match.color == Constants.kRedTarget) {
      return ColorWheelColor.RED;
    } else if (match.color == Constants.kGreenTarget) {
      return ColorWheelColor.GREEN;
    } else if (match.color == Constants.kYellowTarget) {
      return ColorWheelColor.YELLOW;
    } else {
      return ColorWheelColor.UNKNOWN;
    }
  }
  
  /**
   * Cancels Any loops that may be trying to control the spinner, stops wheel, and lowers
   */
  public void stopSpinnerAndLower() {
    setWheelSpeed(0);
    setWheelExtension(SolenoidPosition.DOWN);
  }

  /**
   * Call this function to print details about the color sensor to dashboard
   * You can use these numbers to adjust color calibrations for different 
   * lighting at various events
   */
  public void printColorToDashboardForCal() {
    Color detectedColor = m_colorSensor.getColor();

    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    if (match.color == Constants.kBlueTarget) {
      colorString = "Blue";
    } else if (match.color == Constants.kRedTarget) {
      colorString = "Red";
    } else if (match.color == Constants.kGreenTarget) {
      colorString = "Green";
    } else if (match.color == Constants.kYellowTarget) {
      colorString = "Yellow";
    } else {
      colorString = "Unknown";
    }


    SmartDashboard.putNumber("Red", detectedColor.red);
    SmartDashboard.putNumber("Green", detectedColor.green);
    SmartDashboard.putNumber("Blue", detectedColor.blue);
    SmartDashboard.putNumber("Confidence", match.confidence);
    SmartDashboard.putString("Detected Color", colorString);
  }

  /**
   * Check for game data
   * It's released as soon as one of the alliances
   * charges their shield generator to like level 3
   * I dont actually know
   */
  private void checkForGameData() {
    String gameData = DriverStation.getInstance().getGameSpecificMessage();
    if(gameData.length() > 0){
      switch (gameData.charAt(0)){
        case 'B' :
          this.gameDataColor = ColorWheelColor.BLUE;
          break;
        case 'G' :
          this.gameDataColor = ColorWheelColor.GREEN;
          break;
        case 'R' :
          this.gameDataColor = ColorWheelColor.RED;
          break;
        case 'Y' :
          this.gameDataColor = ColorWheelColor.YELLOW;
          break;
        default :
          this.gameDataColor = ColorWheelColor.UNKNOWN;
          break;
      }
    }
    this.haveGameData = (this.gameDataColor != ColorWheelColor.UNKNOWN);
  }

  /**
   * Hi mom
   * @param pos Position that you want the color wheel thingy to be at
   */
  public void setWheelExtension(Constants.SolenoidPosition pos) {
    if(pos.equals(Constants.SolenoidPosition.UP)) {
      wheelExtension.set(Value.kForward);
    } else if (pos.equals(Constants.SolenoidPosition.DOWN)) {
      wheelExtension.set(Value.kReverse);
    } else {
      wheelExtension.set(Value.kOff);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.checkForGameData();
    SmartDashboard.putBoolean("Have Color Wheel Color?", haveGameData);
    SmartDashboard.putString("Field Wants This Color: ", this.gameDataColor.toString());
  
    //Call The Sensor ONCE per loop
    this.currentSensorColor = getCurrentColor();
    
    currentProx = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Prox", currentProx);
    SmartDashboard.putBoolean("ProxCorrect?", Math.abs(currentProx - Constants.IDEAL_PROX_COLOR_SENSOR) < Constants.PROX_SENSOR_TOLERANCE);

    SmartDashboard.putString("Robot->FieldColor", getFieldColorFromRobotColor(currentSensorColor).toString());
    printColorToDashboardForCal();
  }
}