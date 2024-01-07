// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  
  // Launcher Motor Controllers
  private CANSparkMax m_flyWheel; // NEO motor

  /** Subsystem for controlling the launcher fly wheel */
  public LauncherSubsystem() {
    // Instantiate the fly wheel motor controllers
    m_flyWheel = new CANSparkMax(Constants.FLY_WHEEL_MOTOR_ID, MotorType.kBrushless);

    // Reverse it if needed
    m_flyWheel.setInverted(Constants.FLY_WHEEL_INVERT);

    // Logging
    SmartDashboard.putNumber("Fly Wheel Speed", Constants.FLY_WHEEL_DEFAULT_SPEED);
  }

  /* Set power to the launcher motor */
  public void launch() {
    m_flyWheel.set(SmartDashboard.getNumber("Fly Wheel Speed", Constants.FLY_WHEEL_DEFAULT_SPEED));
  }

  public void stop() {
    m_flyWheel.set(0);
  }

  @Override
  public void periodic() {

  }
}