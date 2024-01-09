// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.commands.autonomous.Drive1MeterAuto;
import frc.robot.commands.autonomous.AutonomousMode_Default;
import frc.robot.commands.autonomous.SquareAutonomous;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import frc.robot.subsystems.PowerSubsystem;
import frc.robot.subsystems.LEDSubsystem.LEDMode;
import frc.robot.subsystems.VisionSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  Command m_autonomousCommand;
	SendableChooser<Command> autonChooser = new SendableChooser<Command>(); // Create a chooser to select an autonomous command

  public static final GenericHID controller = new GenericHID(Constants.CONTROLLER_USB_PORT_ID); // Instantiate our controller at the specified USB port

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem(); // Drivetrain subsystem
  public static final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(); // Intake subsystem
  public static final FeederSubsystem m_feederSubsystem = new FeederSubsystem(); // Feeder subsystem
  public static final ClimberSubsystem m_climbSubsystem = new ClimberSubsystem(); // Climber subsystem
  public static final LauncherSubsystem m_launcherSubsystem = new LauncherSubsystem(); // Launcher subsystem
  public static final PowerSubsystem m_powerSubsystem = new PowerSubsystem(); // Power subsystem for interacting with the Rev PDH
  public static final VisionSubsystem m_visionSubsystem = new VisionSubsystem(); // Subsystem for interacting with Photonvision
  public static final LEDSubsystem m_LEDSubsystem = new LEDSubsystem(); // Subsytem for controlling the REV Blinkin LED module

  boolean launchStarted = false;
  
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {

    // // Starts recording to data log
    // DataLogManager.start();

    // // Record both DS control and joystick data
    // DriverStation.startDataLog(DataLogManager.getLog());

    configureButtonBindings(); // Bind our commands to physical buttons on a controller

    // Add our Autonomous Routines to the chooser //
		autonChooser.setDefaultOption("Default Auto", new AutonomousMode_Default());
    autonChooser.addOption("Square Auto", new SquareAutonomous());
    autonChooser.addOption("Drive 1 Meter", new Drive1MeterAuto());
		SmartDashboard.putData("Auto Mode", autonChooser);

    m_driveSubsystem.setDefaultCommand(new DriveCommand());

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();
  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods. This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Gyroscope Pitch", m_driveSubsystem.getPitch());
    SmartDashboard.putNumber("Gyroscope Yaw", m_driveSubsystem.getYaw());
    SmartDashboard.putNumber("Gyroscope Roll", m_driveSubsystem.getRoll());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // Stops recording to data log
    DataLogManager.stop();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /** This function is called continuously after the robot enters Disabled mode. */
  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Starts recording to data log
    DataLogManager.start();

    System.out.println("AUTONOMOUS MODE STARTED");

    m_autonomousCommand = autonChooser.getSelected();
    
    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // schedule the selected autonomous command
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    // Set the LED pattern for autonomous
    m_LEDSubsystem.setLEDMode(LEDMode.AUTO);
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // Starts recording to data log
    DataLogManager.start();

    // Record both DS control and joystick data
    DriverStation.startDataLog(DataLogManager.getLog());

    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this if statement or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    // Zero the gyro and reset encoders
    m_driveSubsystem.zeroGyro();
    m_driveSubsystem.resetEncoders();

    // Set the LED pattern for teleop
    m_LEDSubsystem.setLEDMode(LEDMode.TELEOP);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    SmartDashboard.putNumber("Controller: Right Trigger", controller.getRawAxis(Constants.RIGHT_TRIGGER_AXIS));
    SmartDashboard.putNumber("Controller: Left Trigger", controller.getRawAxis(Constants.LEFT_TRIGGER_AXIS));
    SmartDashboard.putBoolean("Controller: Right Bumper", controller.getRawButton(Constants.RIGHT_BUMPER));
    SmartDashboard.putBoolean("Controller: Left Bumper", controller.getRawButton(Constants.LEFT_BUMPER));
    SmartDashboard.putBoolean("Controller: X Button", controller.getRawButton(Constants.X_BUTTON));
    SmartDashboard.putBoolean("Controller: Y Button", controller.getRawButton(Constants.Y_BUTTON));
    SmartDashboard.putBoolean("Controller: B Button", controller.getRawButton(Constants.B_BUTTON));
    SmartDashboard.putBoolean("Controller: A Button", controller.getRawButton(Constants.A_BUTTON));
    SmartDashboard.putNumber("Controller: Left Joystick X Axis", controller.getRawAxis(Constants.LEFT_HORIZONTAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Left Joystick Y Axis", controller.getRawAxis(Constants.LEFT_VERTICAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Right Joystick X Axis", controller.getRawAxis(Constants.RIGHT_HORIZONTAL_JOYSTICK_AXIS));
    SmartDashboard.putNumber("Controller: Right Joystick Y Axis", controller.getRawAxis(Constants.RIGHT_VERTICAL_JOYSTICK_AXIS));
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or onse of its subclasses ({@link edu.wpi.first.wpilibj.Joystick} 
   * or {@link XboxController}), and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.Trigger}.
   */
  private void configureButtonBindings() {
    // Drivetrain Controls //
    new Trigger(() -> controller.getRawButton(Constants.Y_BUTTON)).onTrue(new InstantCommand(() -> m_driveSubsystem.toggleDirection())); // Toggle the direction of the drivetrain

    // Climber Controls //
    new POVButton(controller, 0).whileTrue(new StartEndCommand(() -> m_climbSubsystem.setPower(Constants.CLIMBER_SPEED), () -> m_climbSubsystem.stop())); // Climber up
    new POVButton(controller, 180).whileTrue(new StartEndCommand(() -> m_climbSubsystem.setPower(-1 * Constants.CLIMBER_SPEED), () -> m_climbSubsystem.stop())); // Climber down

    // Launcher Controls //
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_TRIGGER_AXIS)).whileTrue(new StartEndCommand(() -> m_launcherSubsystem.launch(), () -> m_launcherSubsystem.stop())); // Launch
    new Trigger(() -> controller.getRawButton(Constants.X_BUTTON)).whileTrue(new FeedCommand(false)); // Feed
    new Trigger(() -> controller.getRawButton(Constants.B_BUTTON)).whileTrue(new FeedCommand(true)); // Unfeed
    new Trigger(() -> controller.getRawButton(Constants.A_BUTTON)).onTrue(new InstantCommand(() -> m_launcherSubsystem.toggleExtension())); // Toggle the launcher extension

    // Intake Controls //
    new Trigger(() -> controller.getRawButton(Constants.RIGHT_BUMPER)).whileTrue(new IntakeCommand(false)); // Intake
    new Trigger(() -> controller.getRawButton(Constants.LEFT_BUMPER)).whileTrue(new IntakeCommand(true)); // Outtake
  }
}