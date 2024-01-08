// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  // Drivetrain Motor Controllers
  private CANSparkMax m_lowerIntakeBar; // NEO 550 motor
  private CANSparkMax m_upperIntakeBar; // NEO 550 motor

  private RelativeEncoder m_lowerIntakeBarEncoder;  // NEO 550 encoder
  private RelativeEncoder m_upperIntakeBarEncoder;  // NEO 550 encoder

  private double lowerIntakeBarRPM, upperIntakeBarRPM;

  // Speed Control Chooser
  SendableChooser<Double> beltSpeedChooser = new SendableChooser<Double>();
  SendableChooser<Double> captureWheelChooser = new SendableChooser<Double>();

  /** Subsystem for controlling the Drivetrain and accessing the NavX Gyroscope */
  public IntakeSubsystem() {
    // Instantiate the Drivetrain motor controllers
    m_lowerIntakeBar = new CANSparkMax(Constants.CAPTURE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    m_upperIntakeBar = new CANSparkMax(Constants.BELT_MOTOR_ID, MotorType.kBrushless);

    // Reverse some of the motors if needed
    m_lowerIntakeBar.setInverted(Constants.CAPTURE_ROLLER_INVERT);
    m_upperIntakeBar.setInverted(Constants.BELT_INVERT);

    m_lowerIntakeBarEncoder = m_lowerIntakeBar.getEncoder();
    m_upperIntakeBarEncoder = m_upperIntakeBar.getEncoder();

    //TODO: Set encoder conversion factor for correct RPM

    // Belt Speed Options //
    captureWheelChooser.addOption("100%", 1.0);
    captureWheelChooser.setDefaultOption("75%", 0.75);
    captureWheelChooser.addOption("50%", 0.5);
    captureWheelChooser.addOption("25%", 0.25);

    // Belt Speed Options //
    beltSpeedChooser.addOption("100%", 1.0);
    beltSpeedChooser.setDefaultOption("75%", 0.75);
    beltSpeedChooser.addOption("50%", 0.5);
    beltSpeedChooser.addOption("25%", 0.25);

    SmartDashboard.putData("Capture Wheel Speed", captureWheelChooser);
    SmartDashboard.putData("Belt Speed", beltSpeedChooser);
  }

  /* Set power to the drivetrain motor */
  public void capture() {
    m_lowerIntakeBar.set(captureWheelChooser.getSelected());
  }

  public void stopCapture() {
    m_lowerIntakeBar.set(0);
  }

  public void feed() {
    m_upperIntakeBar.set(beltSpeedChooser.getSelected());
  }

  public void reverseFeed() {
    m_upperIntakeBar.set(beltSpeedChooser.getSelected());
  }

  public void stopFeed() {
    m_upperIntakeBar.set(0);
  }

  public void setCapturePower(double power) {
    m_lowerIntakeBar.set(power);
  }

  public void setBeltPower(double power) {
    m_upperIntakeBar.set(power);
  }

  public double getLowerIntakeBarRPM() {
    return lowerIntakeBarRPM;
  }

  public double getUpperIntakeBarRPM() {
    return upperIntakeBarRPM;
  }

  public void stop() {
    m_lowerIntakeBar.set(0);
    m_upperIntakeBar.set(0);
  }

  @Override
  public void periodic() {
    lowerIntakeBarRPM = m_lowerIntakeBarEncoder.getVelocity();
    upperIntakeBarRPM = m_upperIntakeBarEncoder.getVelocity();

    // Add intake bar RPMs to SmartDashboard for the sake of datalogging
    SmartDashboard.putNumber("Lower Intake Bar RPM", lowerIntakeBarRPM);
    SmartDashboard.putNumber("Upper Intake Bar RPM", upperIntakeBarRPM);
  }
}