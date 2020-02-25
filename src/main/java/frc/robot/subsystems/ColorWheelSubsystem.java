/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Enum for Colors
// .toString() returns the name, so RED.toString() returns "RED"
enum ColorWheelColor { 
    RED, GREEN, BLUE, YELLOW, UNKNOWN
} 
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

  // Boolean to enable/disable loops that control the spinner motor
  private boolean runSpinnerAutonomously = false;

  // Boolean to keep track if spinner is running autonomously
  private boolean isSpinnerRunningAutonomously = false;

  // Boolean to track if we have the Game Data
  private boolean haveGameData = false;

  // Store Game Data (if we have it)
  private ColorWheelColor gameDataColor = ColorWheelColor.UNKNOWN;

  // Current Color of sensor (Read once once per loop)
  private ColorWheelColor currentSensorColor = ColorWheelColor.UNKNOWN;
  
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

  public void moveToColor(ColorWheelColor color) {
    // Cancel Other loops, if running
    runSpinnerAutonomously = false; 
    // Set that we are auton controlling this motor here
    runSpinnerAutonomously = true; 
    // Start a new thread so we don't bog down the main thread
    new Thread() {
      public void run() {
        // Start Spinning motor
        colorWheelMotor.set(ControlMode.PercentOutput, Constants.COLOR_WHEEL_SPIN_SPEED);
        // Wait Untill Desired Color is found
        while(getFieldColorFromRobotColor(currentSensorColor) != color && runSpinnerAutonomously) { 
          isSpinnerRunningAutonomously = true;
        }
        // While Loop Has exited, either because we finished or because it's been canceled. Stop the motor
        colorWheelMotor.set(ControlMode.PercentOutput, 0);
        // Set that we aren't running motor autoniomously
        isSpinnerRunningAutonomously = false;
      }
    }.start();
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
   * We can Gain position Control by rotating the color wheel to the field data
   */
  public void gainPositionControl(boolean alreadyInAThread) {
    // If we dont have the game data or the game data is unknown, exit function.
    if(!this.haveGameData || gameDataColor == ColorWheelColor.UNKNOWN) return;
    if(alreadyInAThread) {
      // If we're already in a thread, we don't need to launch a new one
      int seenDesiredColor = 0;
      colorWheelMotor.set(ControlMode.PercentOutput, Constants.COLOR_WHEEL_SPIN_SPEED * .5);
      // Wait for Color Sensor to return the right value
      while(runSpinnerAutonomously && seenDesiredColor < 2) {
        SmartDashboard.putNumber("PosCtrlDesColNum", seenDesiredColor);
        if(getFieldColorFromRobotColor(currentSensorColor) == gameDataColor) {
          seenDesiredColor++;
        }
        isSpinnerRunningAutonomously = true;
      }
      // Stop Spinning
      colorWheelMotor.set(ControlMode.PercentOutput, 0);
      // Update these
      isSpinnerRunningAutonomously = false;
      runSpinnerAutonomously = false;
      // Retract Color Wheel Spinner that spins the wheel
      setWheelExtension(Constants.SolenoidPosition.DOWN);
    } else {
      // Create a new thread
      // Cancel Other Loops
      this.runSpinnerAutonomously = false;
      // Set that we're running now
      this.runSpinnerAutonomously = true;
      // Create a new thread so we don't bog down the main thread
      new Thread() {
        public void run() {
          // Extend Color Wheel Wheel Spinner that spins the wheel
          setWheelExtension(Constants.SolenoidPosition.UP);
          // Start Spinning
          colorWheelMotor.set(ControlMode.PercentOutput, Constants.COLOR_WHEEL_SPIN_SPEED);
          // Wait for Color Sensor to return the right value
          while(getFieldColorFromRobotColor(currentSensorColor) != gameDataColor && runSpinnerAutonomously) {
            isSpinnerRunningAutonomously = true;
          }
          // Stop Spinning
          colorWheelMotor.set(ControlMode.PercentOutput, 0);
          // Update these
          isSpinnerRunningAutonomously = false;
          runSpinnerAutonomously = false;
          // Retract Color Wheel Spinner that spins the wheel
          setWheelExtension(Constants.SolenoidPosition.DOWN);
        }
      }.start();
    }
  }

  /**
   * We can Gain rotation Control by rotating the color wheel 3 times
   */
  public void gainRotationControl(boolean gainPosControlAfter) {
    // Cancel Other Loops
    this.runSpinnerAutonomously = false;
    // Set that we're running now
    this.runSpinnerAutonomously = true;
    // Create a new thread so we don't bog down the main thread
    new Thread() {
      public void run() {
        // Extend Color Wheel Spinner that spins the wheel
        setWheelExtension(Constants.SolenoidPosition.UP);
        // Start Spinning
        colorWheelMotor.set(ControlMode.PercentOutput, Constants.COLOR_WHEEL_SPIN_SPEED);
        // SmartDashboard Update
        SmartDashboard.putString("StartColor", "Have Not Read Initial Color");
        // Wait for Prox sensor To Detect Color Wheel
        while(Math.abs(currentProx - Constants.IDEAL_PROX_COLOR_SENSOR) > Constants.PROX_SENSOR_TOLERANCE){}
        // Get Start Color
        ColorWheelColor startColor = currentSensorColor;
        // SmartDashboard Update
        SmartDashboard.putString("StartColor", currentSensorColor.toString());
        
        // Variables be used in the loop
        ColorWheelColor lastColor = currentSensorColor;
        int numberOfTimesSeenStartColor = 0;
        while(runSpinnerAutonomously) {
          isSpinnerRunningAutonomously = true;
          if(lastColor != currentSensorColor) {
            //New Color Detected
            if(currentSensorColor == startColor) {
              // We see the start color, lets add it
              numberOfTimesSeenStartColor++;
            }
            // Update last color with the current color
            lastColor = currentSensorColor;
            // SmartDashboard Update
            SmartDashboard.putNumber("NumSeenStartColor", numberOfTimesSeenStartColor);
            // If we've seen the start color 6 times, we've rotated it 3 times. We can stop.
            if(numberOfTimesSeenStartColor == 6) break; 
          }
        }
        if(gainPosControlAfter) {
          //colorWheelMotor.set(ControlMode.PercentOutput, 0);
          //setWheelExtension(Constants.SolenoidPosition.DOWN);
          //SmartDashboard.putString("ColorStoppedOn", currentSensorColor.toString());
          gainPositionControl(true);
        } else {
          // Stop Spinning
          colorWheelMotor.set(ControlMode.PercentOutput, 0);
          // Retract Color Wheel Spinner that spins the wheel
          setWheelExtension(Constants.SolenoidPosition.DOWN);
          // Update these
          isSpinnerRunningAutonomously = false;
          runSpinnerAutonomously = false;
        }
      }
    }.start();
  }

  /**
   * Get The Currently Detect Color at the Sensor
   * @return ColorWheelColor Detected Color
   */
  private ColorWheelColor getCurrentColor() {
    currentProx = m_colorSensor.getProximity();
    SmartDashboard.putNumber("Prox", currentProx);
    SmartDashboard.putBoolean("ProxCorrect?", Math.abs(currentProx - Constants.IDEAL_PROX_COLOR_SENSOR) < Constants.PROX_SENSOR_TOLERANCE);
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
   * Cancels Any loops that may be trying to control the spinner
   */
  public void cancelSpinnerIfRunning() {
    runSpinnerAutonomously = false;
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
   * @return If the spinner is being controled in a loop somewhere
   */
  public boolean getIsSpinnerRunningAutononously() {
    return isSpinnerRunningAutonomously;
  }

  /**
   * Stop the spinner, if it's running in a loop somewhere
   */
  public void stopSpinner() {
    this.runSpinnerAutonomously = false;
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
    SmartDashboard.putString("Robot->FieldColor", getFieldColorFromRobotColor(currentSensorColor).toString());
    printColorToDashboardForCal();
  }
}