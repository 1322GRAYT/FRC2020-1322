/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final AimSubsystem aimSubsystem = new AimSubsystem();
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();
  private final ShiftSubsystem shiftSubsystem = new ShiftSubsystem();
  
  private XboxController driverStick;
  private XboxController auxStick;

  private TurretTrigger turretTrigger = new TurretTrigger();
  
  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);


  class TurretTrigger extends Trigger{
    @Override
    public boolean get() {
      return visionSubsystem.hasTarget();
    }
  }

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    // Configure Default Commands
    setDefaultCommands();
    // Set a trigger (eww, they're depricated)
    turretTrigger.whileActiveContinuous(new AutoAim(aimSubsystem, visionSubsystem, auxStick));
  }

  private void setDefaultCommands() {
    CommandScheduler.getInstance().setDefaultCommand(liftSubsystem, new ManualLift(liftSubsystem, driverStick));
    CommandScheduler.getInstance().setDefaultCommand(aimSubsystem, new ManualTurret(aimSubsystem, auxStick));
    CommandScheduler.getInstance().setDefaultCommand(ballSubsystem, new ManualIntakeLift(ballSubsystem, auxStick));
    //CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, new BreakInDrive(driveSubsystem, driverStick));
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* BEGIN DRIVER STICK BUTTON ASSIGNMENTS */
    driverStick = new XboxController(0);
    // Shifter Command (Y for high gear, X for low gear)
    new JoystickButton(driverStick, 4).whenPressed(new ShiftCommand(shiftSubsystem, DriveShiftPos.HIGH_GEAR));
    new JoystickButton(driverStick, 1).whenPressed(new ShiftCommand(shiftSubsystem, DriveShiftPos.LOW_GEAR));

    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */
    auxStick = new XboxController(1);
    // Manual Shoot
    new JoystickButton(auxStick, 3).whenPressed(new ManualShoot(ballSubsystem, turretSubsystem, auxStick));
    // Color Wheel Commands (Start for Gain All control, Y to cancel)
    new JoystickButton(auxStick, 8).whenPressed(new GainFullColorWheelCtrl(colorWheelSubsystem));
    //new JoystickButton(auxStick, 8).whenPressed(new ColorWheelCommand(colorWheelSubsystem, ColorWheelCommandExecute.GAIN_BOTH_SAME_TIME));
    //new JoystickButton(auxStick, 4).whenPressed(new ColorWheelCommand(colorWheelSubsystem, ColorWheelCommandExecute.STOP_ALL));

    // Ball Suck Command (Bumpers)
    //new JoystickButton(auxStick, 5).whenPressed(new BallSuckCommand(ballSubsystem, -1));
    //new JoystickButton(auxStick, 5).whenReleased(new BallSuckCommand(ballSubsystem, 0));
    //new JoystickButton(auxStick, 6).whenPressed(new BallSuckCommand(ballSubsystem, 1));
    //new JoystickButton(auxStick, 6).whenReleased(new BallSuckCommand(ballSubsystem, 0));
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
