// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FeederSubsystem extends SubsystemBase {
  
  // Feeder Motor Controller
  private CANSparkMax m_feeder; // NEO motor

  // Feeder Limit Switch
  DigitalInput feederLimitSwitch = new DigitalInput(4);

  /** Subsystem for controlling the Feeder */
  public FeederSubsystem() {
    // Instantiate the feeder motor controller
    m_feeder = new CANSparkMax(Constants.FEEDER_WHEEL_MOTOR_ID, MotorType.kBrushless);

    // Reverse it if needed
    m_feeder.setInverted(Constants.FEEDER_WHEEL_INVERT);

    // Put the default speed on SmartDashboard
    SmartDashboard.putNumber("Feeder Speed", Constants.FEEDER_WHEEL_SPEED);
  }

  /* Set power to the feeder motor */
  public void setPower(double power) {
    m_feeder.set(power);
  }
  public void stop() {
    setPower(0);
  }

  public boolean getFeederLimitSwitch() {
    return feederLimitSwitch.get();
  }

  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Feeder Limit Switch", getFeederLimitSwitch());
  }
}