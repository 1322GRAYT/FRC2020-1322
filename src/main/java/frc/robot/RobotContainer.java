/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.ColorWheelCommandExecute;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final TurretSubsystem shooterSubsystem = new TurretSubsystem();
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  
  private XboxController driverStick;
  private XboxController auxStick;
  
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);



  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(liftSubsystem, new ManualLift(liftSubsystem, auxStick));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    driverStick = new XboxController(0);
    auxStick = new XboxController(1);
    // Primitive Autoaim Command
    new JoystickButton(auxStick, 1).whenPressed(new AutoAimCommand(shooterSubsystem, visionSubsystem));
    // Ball Suck Command (Bumpers)
    new JoystickButton(auxStick, 5).whenPressed(new BallSuckCommand(ballSubsystem, -1));
    new JoystickButton(auxStick, 5).whenReleased(new BallSuckCommand(ballSubsystem, 0));
    new JoystickButton(auxStick, 6).whenPressed(new BallSuckCommand(ballSubsystem, 1));
    new JoystickButton(auxStick, 6).whenReleased(new BallSuckCommand(ballSubsystem, 0));
    // Manual Shoot
    new JoystickButton(auxStick, 3).whenPressed(new ManualShootCommand(ballSubsystem, shooterSubsystem));
    // Color Wheel Commands (Start for Gain Pos Control, Select for Gain Rot Control)
    new JoystickButton(auxStick, 7).whenPressed(new ColorWheelCommand(colorWheelSubsystem, ColorWheelCommandExecute.GAIN_POS_CONTROL));
    new JoystickButton(auxStick, 8).whenPressed(new ColorWheelCommand(colorWheelSubsystem, ColorWheelCommandExecute.GAIN_ROT_CONTROL));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }*/
}
