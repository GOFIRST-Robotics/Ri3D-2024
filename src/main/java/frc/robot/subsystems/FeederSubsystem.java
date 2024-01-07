// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  
  // Drivetrain Motor Controllers
  private CANSparkMax m_captureRoller; // NEO 550 motor
  private CANSparkMax m_belt; // NEO 550 motor

  // Speed Control Chooser
  SendableChooser<Double> beltSpeedChooser = new SendableChooser<Double>();
  SendableChooser<Double> captureWheelChooser = new SendableChooser<Double>();

  /** Subsystem for controlling the Drivetrain and accessing the NavX Gyroscope */
  public FeederSubsystem() {
    // Instantiate the Drivetrain motor controllers
    m_captureRoller = new CANSparkMax(Constants.CAPTURE_ROLLER_MOTOR_ID, MotorType.kBrushless);
    m_belt = new CANSparkMax(Constants.BELT_MOTOR_ID, MotorType.kBrushless);

    // Reverse some of the motors if needed
    m_captureRoller.setInverted(Constants.CAPTURE_ROLLER_INVERT);
    m_belt.setInverted(Constants.BELT_INVERT);

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
    m_captureRoller.set(captureWheelChooser.getSelected());
  }

  public void stopCapture() {
    m_captureRoller.set(0);
  }

  public void feed() {
    m_belt.set(beltSpeedChooser.getSelected());
  }

  public void reverseFeed() {
    m_belt.set(beltSpeedChooser.getSelected());
  }

  public void stopFeed() {
    m_belt.set(0);
  }

  public void stop() {
    m_captureRoller.set(0);
    m_belt.set(0);
  }



  @Override
  public void periodic() {
  }

}