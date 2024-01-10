// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  
  // Climber Motor Controllers
  private VictorSP m_climber; // 775pro motor

  /** Subsystem for controlling the climber */
  public ClimberSubsystem() {


    m_climber = new VictorSP(Constants.CLIMBER_MOTOR_ID_DRIVETRAIN);

    m_climber.setInverted(Constants.CLIMBER_INVERT);



    // Instantiate the climber motor controller
    // m_climber = new TalonSRX(Constants.CLIMBER_MOTOR_ID);
    // m_climber.configFactoryDefault();
    // m_climber.setNeutralMode(NeutralMode.Brake);


    // Reverse it if needed
    // m_climber.setInverted(Constants.CLIMBER_INVERT);

    // Put the default speed on SmartDashboard
    SmartDashboard.putNumber("Climber Speed", Constants.CLIMBER_SPEED);
  }

  /* Set power to the climber motor */
  public void setPower(double power) {

    m_climber.set(power);
    // m_climber.set(TalonSRXControlMode.PercentOutput, power);
  }
  public void stop() {
    setPower(0);
  }

  @Override
  public void periodic() {}
}