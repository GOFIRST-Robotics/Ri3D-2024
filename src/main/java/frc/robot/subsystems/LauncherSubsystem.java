// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  
  // Launcher Motor Controllers
  private CANSparkMax m_flyWheel; // NEO motor
  private TalonSRX m_feederWheel; // BAG DC motor

  /** Subsystem for controlling the launcher fly wheel */
  public LauncherSubsystem() {
    // Instantiate the launcher motor controllers
    m_flyWheel = new CANSparkMax(Constants.FLY_WHEEL_MOTOR_ID, MotorType.kBrushless);
    m_feederWheel = new TalonSRX(Constants.FEEDER_WHEEL_MOTOR_ID);

    // Reverse it if needed
    m_flyWheel.setInverted(Constants.FLY_WHEEL_INVERT);
    m_feederWheel.setInverted(Constants.FEEDER_WHEEL_INVERT);

    // configure Talon SRX motor controllers
    m_feederWheel.configFactoryDefault();
    m_feederWheel.setNeutralMode(NeutralMode.Brake);

    // Logging
    SmartDashboard.putNumber("Fly Wheel Speed", Constants.FLY_WHEEL_DEFAULT_SPEED);
  }

  /* Set power to the launcher motor */
  public void launch() {
    m_flyWheel.set(SmartDashboard.getNumber("Fly Wheel Speed", Constants.FLY_WHEEL_DEFAULT_SPEED));
    m_feederWheel.set(TalonSRXControlMode.PercentOutput, Constants.FEEDER_WHEEL_DEFAULT_SPEED);
  }

  public void flyWheelPower(double power) {
    m_flyWheel.set(power);
  }

  public void feederWheelPower(double power) {
    m_feederWheel.set(TalonSRXControlMode.PercentOutput, power);
  }

  public double getFlyWheelSpeed() {
    return m_flyWheel.getEncoder().getVelocity();
  }

  public void stop() {
    m_flyWheel.set(0);
    m_feederWheel.set(TalonSRXControlMode.PercentOutput, 0);
  }

  @Override
  public void periodic() {

  }
}