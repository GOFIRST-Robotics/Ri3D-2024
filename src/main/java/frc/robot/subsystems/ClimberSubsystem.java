// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Climber Motor Controllers
  private CANSparkMax m_climber; // NEO motor

  // Variables for encoder PID
  public int currentSetpoint;


  /** Subsystem for controlling the climber */
  public ClimberSubsystem() {
    // Instantiate the climber motor controllers
    m_climber = new CANSparkMax(Constants.CLIMBER_MOTOR_ID, MotorType.kBrushless);
    

    // Reverse it if needed
    m_climber.setInverted(Constants.CLIMBER_INVERT);

    // 
    SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_DEFAULT_SPEED);
  }

  /* Set power to the climber motor */
  public void launch() {
    m_climber.set(SmartDashboard.getNumber("Climber Speed", Constants.CLIMBER_DEFAULT_SPEED));
  }

  public void stop() {
    m_climber.set(0);
  }

  // Methods for changing the setpoint/goal of this subsystem's default command //
  public int getCurrentSetPoint() {
    return currentSetpoint;
  }
  public void changeSetpoint(int newSetPoint) {
    if (newSetPoint <= 4 && newSetPoint >= 0) {
      currentSetpoint = newSetPoint;
    }
  }
  public void incrementSetPoint() {
    currentSetpoint = Math.min(currentSetpoint + 1, 4);
  }
  public void decrementSetPoint() {
    currentSetpoint = Math.max(currentSetpoint - 1, 0);
  }

  @Override
  public void periodic() {

  }
}