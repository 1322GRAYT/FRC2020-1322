/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants;
import frc.robot.commands.*;
import frc.robot.commandgroups.*;
import frc.robot.subsystems.*;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
  public  final pRFSLIB prfsLIB = new pRFSLIB();
  private final SendableChooser<Command> m_chooser = new SendableChooser<Command>();
  private final DriveSubsystem driveSubsystem = new DriveSubsystem();
  private final ColorWheelSubsystem colorWheelSubsystem = new ColorWheelSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();
  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final AimSubsystem aimSubsystem = new AimSubsystem();
  private final BallSubsystem ballSubsystem = new BallSubsystem();
  private final LiftSubsystem liftSubsystem = new LiftSubsystem();

  private XboxController driverStick;
  private XboxController auxStick;

  private TurretTrigger turretTrigger = new TurretTrigger();

  private Command m_autoSelected;
  
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
    // Configure Autonomous Selections Available
    m_chooser.setDefaultOption("Default Auto", new CG_InitRobot(driveSubsystem));
    m_chooser.addOption("Do Nothing", new CG_InitRobot(driveSubsystem));
    m_chooser.addOption("Just Shoot", new ManualShoot(ballSubsystem, turretSubsystem, auxStick));
    m_chooser.addOption("Drive Back 3 Ft", new CG_AutoDrvBack3(driveSubsystem));
    m_chooser.addOption("Shoot and Drive Back 3 Ft", new CG_AutoShootDrvBack3(driveSubsystem, ballSubsystem, turretSubsystem, auxStick));
    SmartDashboard.putData("Auto choices: ", m_chooser);

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
    CommandScheduler.getInstance().setDefaultCommand(driveSubsystem, new DRV_DrvManual(driveSubsystem, driverStick));
  }

  public DriveSubsystem getDriveSubsystem()
    {
    return(driveSubsystem);
    }    


  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* BEGIN DRIVER STICK BUTTON ASSIGNMENTS */
    driverStick = new XboxController(Constants.DRVR_CNTRLR);

    
    /* BEGIN AUXILLARY STICK BUTTON ASSIGNMENTS */
    auxStick = new XboxController(Constants.AUX_CNTRLR);
    // Manual Shoot
    new JoystickButton(auxStick, Constants.BUTTON_X).whenPressed(new ManualShoot(ballSubsystem, turretSubsystem, auxStick));
    // Color Wheel Commands (Start for Gain All control)
    new JoystickButton(auxStick, Constants.BUTTON_START).whenPressed(new GainFullColorWheelCtrl(colorWheelSubsystem));
    new JoystickButton(auxStick, Constants.BUTTON_BACK).whenPressed(new RetractColorWheel(colorWheelSubsystem));
  }


  /**
   * Use this to pass the Autonomous Command(S) for Autonomous Init to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousInitCommand() {
      // An ExampleCommand will run in autonomous
      m_autoSelected = m_chooser.getSelected(); 
      System.out.println("Auto selected: " + m_autoSelected);
      return(m_autoSelected);
  }


  /**
   * Use this to pass the Autonomous Command(s) for Periodic Autonomous to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
/*  
  public Command getAutonomousPeriodicCommand() {
      // An ExampleCommand will run in autonomous
      return(new ManualShoot(ballSubsystem, turretSubsystem, auxStick));
  }
*/


}
