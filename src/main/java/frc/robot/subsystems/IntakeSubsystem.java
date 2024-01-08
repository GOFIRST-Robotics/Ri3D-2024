// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  // Intake Motor Controllers
  private CANSparkMax m_lowerIntakeBar; // NEO 550 motor
  private CANSparkMax m_upperIntakeBar; // NEO 550 motor

  private RelativeEncoder m_lowerIntakeBarEncoder;  // NEO 550 encoder
  private RelativeEncoder m_upperIntakeBarEncoder;  // NEO 550 encoder

  private double lowerIntakeBarRPM, upperIntakeBarRPM;

  /** Subsystem for controlling the Intake */
  public IntakeSubsystem() {
    // Instantiate the Intake motor controllers
    m_lowerIntakeBar = new CANSparkMax(Constants.LOWER_INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);
    m_upperIntakeBar = new CANSparkMax(Constants.UPPER_INTAKE_BAR_MOTOR_ID, MotorType.kBrushless);

    // Reverse some of the motors if needed
    m_lowerIntakeBar.setInverted(Constants.LOWER_INTAKE_BAR_INVERT);
    m_upperIntakeBar.setInverted(Constants.UPPER_INTAKE_BAR_INVERT);

    m_lowerIntakeBarEncoder = m_lowerIntakeBar.getEncoder();
    m_upperIntakeBarEncoder = m_upperIntakeBar.getEncoder();

    //TODO: Set encoder conversion factor for correct RPM

    SmartDashboard.putNumber("Upper Intake Bar Speed", Constants.UPPER_INTAKE_BAR_DEFAULT_SPEED);
    SmartDashboard.putNumber("Lower Intake Bar Speed", Constants.LOWER_INTAKE_BAR_DEFAULT_SPEED);
  }

  /* Set power to the intake motors */
  public void setPower(int reverse) {
    m_upperIntakeBar.set(reverse * Constants.UPPER_INTAKE_BAR_DEFAULT_SPEED);
    m_lowerIntakeBar.set(reverse * Constants.LOWER_INTAKE_BAR_DEFAULT_SPEED);
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