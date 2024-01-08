// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  
  // Launcher Motor Controllers
  private CANSparkMax m_flyWheel; // NEO motor
  private TalonSRX m_feederWheel; // BAG DC motor

  private SparkPIDController m_flyWheelPIDController;
  private RelativeEncoder m_flyWheelEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double power;
  private double flyWheelRPM, flyWheelTargetRPM;

  private boolean isExtended; // This variable keeps track of whether the grabber piston is currently extended or not
  private DoubleSolenoid extensionSolenoid; // A double solenoid takes up two PCM channels

  /** Subsystem for controlling the launcher fly wheel */
  public LauncherSubsystem() {
    configureFlyWheel();
    configureFeederWheel();

    extensionSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.EXTENSION_SOLENOID_ID_1, Constants.EXTENSION_SOLENOID_ID_2);
    isExtended = false;
  }

  public void configureFlyWheel() {
    m_flyWheel = new CANSparkMax(Constants.FLY_WHEEL_MOTOR_ID, MotorType.kBrushless);

    /**
     * The RestoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_flyWheel.restoreFactoryDefaults();

    m_flyWheel.setInverted(Constants.FLY_WHEEL_INVERT);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_flyWheelPIDController = m_flyWheel.getPIDController();

    // Encoder object created to display position values
    m_flyWheelEncoder = m_flyWheel.getEncoder();

    // PID coefficients
    kP = 6e-5; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.000015; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

    flyWheelRPM = 0;
    flyWheelTargetRPM = 0;

    // set PID coefficients
    m_flyWheelPIDController.setP(kP);
    m_flyWheelPIDController.setI(kI);
    m_flyWheelPIDController.setD(kD);
    m_flyWheelPIDController.setIZone(kIz);
    m_flyWheelPIDController.setFF(kFF);
    m_flyWheelPIDController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("Flywheel P Gain", kP);
    SmartDashboard.putNumber("Flywheel I Gain", kI);
    SmartDashboard.putNumber("Flywheel D Gain", kD);
    SmartDashboard.putNumber("Flywheel I Zone", kIz);
    SmartDashboard.putNumber("Flywheel Feed Forward", kFF);
    SmartDashboard.putNumber("Flywheel Max Output", kMaxOutput);
    SmartDashboard.putNumber("Flywheel Min Output", kMinOutput);

    // Logging
    SmartDashboard.putNumber("Fly Wheel RPM", flyWheelRPM);
    SmartDashboard.putNumber("Fly Wheel Target RPM", flyWheelTargetRPM);
  }

  public void configureFeederWheel() {
    // Instantiate the launcher motor controllers
    m_feederWheel = new TalonSRX(Constants.FEEDER_WHEEL_MOTOR_ID);

    // Reverse it if needed
    m_feederWheel.setInverted(Constants.FEEDER_WHEEL_INVERT);

    // configure Talon SRX motor controllers
    m_feederWheel.configFactoryDefault();
    m_feederWheel.setNeutralMode(NeutralMode.Brake);
  }

  /* Set power to the launcher motor */
  public void launch() {
    m_flyWheel.set(SmartDashboard.getNumber("Fly Wheel Speed", Constants.FLY_WHEEL_SPEED));
    m_feederWheel.set(TalonSRXControlMode.PercentOutput, Constants.FEEDER_WHEEL_SPEED);
  }
  
  public void flyWheelPower(double power) {
    // m_flyWheel.set(power);
    this.power = power;

    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double setPoint = power*maxRPM;
    m_flyWheelPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setFlyWheelRPM(double RPM) {
    flyWheelTargetRPM = RPM;
    m_flyWheelPIDController.setReference(RPM, CANSparkMax.ControlType.kVelocity);
  }

  public void feederWheelPower(double power) {
    m_feederWheel.set(TalonSRXControlMode.PercentOutput, power);
  }

  public double getFlyWheelRPM() {
    return flyWheelRPM;
  }

  public double getFlyWheelTargetRPM() {
    return flyWheelTargetRPM;
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
    flyWheelPIDSmartDashboard();
  }
  
  public void flyWheelPIDSmartDashboard() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_flyWheelPIDController.setP(p); kP = p; }
    if((i != kI)) { m_flyWheelPIDController.setI(i); kI = i; }
    if((d != kD)) { m_flyWheelPIDController.setD(d); kD = d; }
    if((iz != kIz)) { m_flyWheelPIDController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_flyWheelPIDController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_flyWheelPIDController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }

    double inputTargetRPM = SmartDashboard.getNumber("Fly Wheel Target RPM", 0);

    if (inputTargetRPM != flyWheelTargetRPM) {
      setFlyWheelRPM(inputTargetRPM);
    }
    
    flyWheelRPM = m_flyWheelEncoder.getVelocity();

    SmartDashboard.putNumber("Power", power);
    SmartDashboard.putNumber("Fly Wheel RPM", flyWheelRPM);
  }
}