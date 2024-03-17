// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Climber Motor Controllers
  // private VictorSP m_climber; // 775pro motor
  // private TalonSRX m_climber; // 775pro motor
  private CANSparkMax m_climber; // NEO motor

  /** Subsystem for controlling the climber */
  public ClimberSubsystem() {

    // Setup if useing VictorSP motor controller
    // m_climber = new VictorSP(Constants.CLIMBER_MOTOR_ID_DRIVETRAIN);
    // m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Setup if using TalonSRX motor controller
    // m_climber = new TalonSRX(Constants.CLIMBER_MOTOR_ID_TALON_SRX);
    // m_climber.configFactoryDefault();
    // m_climber.setNeutralMode(NeutralMode.Brake);

    // Setup using SparkMax motor controller
    m_climber = new CANSparkMax(Constants.CLIMBER_MOTOR_ID_SPARK, MotorType.kBrushed);
    m_climber.restoreFactoryDefaults();
    m_climber.setIdleMode(IdleMode.kBrake);
    m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Reverse it if needed
    // m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Put the default speed on SmartDashboard
    // SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_SPEED);
  }

  /* Set power to the climber motor */
  public void setPower(double power) {
    // Victor SP set method
    // m_climber.set(power);

    // Talon SRX set method
    // m_climber.set(TalonSRXControlMode.PercentOutput, power);

    // Spark Max set method
    m_climber.set(power);
  }
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}