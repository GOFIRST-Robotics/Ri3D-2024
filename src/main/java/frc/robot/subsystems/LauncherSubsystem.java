// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.subsystems;

import frc.robot.Constants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LauncherSubsystem extends SubsystemBase {
  
  // Launcher Motor Controllers
  private CANSparkMax m_flyWheel; // NEO motor

  private Compressor m_compressor;
  private PowerSubsystem m_powerSubsystem;
  private SparkMaxPIDController m_flyWheelPIDController;
  private RelativeEncoder m_flyWheelEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  private double power;
  private double flyWheelRPM, flyWheelTargetRPM;
  private boolean setFlyWheelRPMSuccess; 
  private int flyWheelPower;
  public int manualRPMSet;
  public double flyWheelTargetPower;

  private boolean isExtended; // This variable keeps track of whether the piston is currently extended or not
  private Solenoid extensionSolenoid;

  /** Subsystem for controlling the launcher fly wheel */
  public LauncherSubsystem(PowerSubsystem powerSubsystem) {
    configureFlyWheel();
    m_powerSubsystem = powerSubsystem;

    m_compressor = new Compressor(Constants.COMPRESSOR_CAN_ID, PneumaticsModuleType.REVPH);

    extensionSolenoid = new Solenoid(PneumaticsModuleType.REVPH, Constants.EXTENSION_SOLENOID_ID);
    isExtended = false;
    manualRPMSet = 0;

    SmartDashboard.putBoolean("Launcher Extended", isExtended);

    m_compressor.enableDigital();
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

    // Encoder object created to display position values
    m_flyWheelEncoder = m_flyWheel.getEncoder();

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_flyWheelPIDController = m_flyWheel.getPIDController();

    // PID coefficients
    kP = Constants.FLY_WHEEL_P;
    kI = Constants.FLY_WHEEL_I;
    kD = Constants.FLY_WHEEL_D;
    kIz = Constants.FLY_WHEEL_IZONE;
    kFF = Constants.FLY_WHEEL_FF;
    kMaxOutput = Constants.FLY_WHEEL_K_MAX_OUTPUT;
    kMinOutput = Constants.FLY_WHEEL_K_MIN_OUTPUT;
    maxRPM = Constants.FLY_WHEEL_MAX_RPM;

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
    SmartDashboard.putNumber("Manual RPM set", manualRPMSet);
    SmartDashboard.putNumber("Power", power);
    SmartDashboard.putNumber("FlyWheel Target Power", flyWheelTargetPower);
  }

  /* Set power to the launcher motor */
  public void launch() {
    //m_flyWheel.set(SmartDashboard.getNumber("Fly Wheel Speed", Constants.FLY_WHEEL_SPEED));
    // setFlyWheelRPM(Constants.FLY_WHEEL_SPEED);
    //m_feederWheel.set(TalonSRXControlMode.PercentOutput, Constants.FEEDER_WHEEL_SPEED);
  }

  /* Solenoid Methods */
  public void extend() {
    // When using PDH switched channel for solenoid
    m_powerSubsystem.setSwitchedChannel(false); //  Logic is inverted

    // When using REV PH solenoid
    extensionSolenoid.set(true);

    isExtended = true;
    SmartDashboard.putBoolean("Launcher Extended", isExtended);
  }
  public void retract() {
    // When using PDH switched channel for solenoid
    m_powerSubsystem.setSwitchedChannel(true); // Logic is inverted

    // When using REV PH solenoid
    extensionSolenoid.set(false);

    isExtended = false;
    SmartDashboard.putBoolean("Launcher Extended", isExtended);
  }
  public void toggleExtension() {
    if (isExtended) {
      retract();
    } else {
      extend();
    }
  }
  
  public void setFlyWheelPower(double power) {
    if (Math.abs(power) > Constants.FLY_WHEEL_MAX_POWER) {
      power = Math.copySign(Constants.FLY_WHEEL_MAX_POWER, power); 
    }

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
    m_flyWheel.set(power);
    // double setPoint = power*maxRPM;
    // m_flyWheelPIDController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);
  }

  public void setFlyWheelRPM(double RPM) {
    if (Math.abs(RPM) > Constants.FLY_WHEEL_MAX_RPM) {
      RPM = Math.copySign(Constants.FLY_WHEEL_MAX_RPM, RPM); 
    }

    flyWheelTargetRPM = RPM;
    if(m_flyWheelPIDController.setReference(RPM, CANSparkMax.ControlType.kVelocity).equals(REVLibError.kOk)) {
      setFlyWheelRPMSuccess = true;
    }
    setFlyWheelRPMSuccess = false;

    SmartDashboard.putNumber("PID Output Max", m_flyWheelPIDController.getOutputMax());
    SmartDashboard.putNumber("PID Output Min", m_flyWheelPIDController.getOutputMin());

    SmartDashboard.putBoolean("Set Fly Wheel RPM Success", setFlyWheelRPMSuccess);
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
  }

  @Override
  public void periodic() {
    flyWheelPIDSmartDashboard();
  }
  
  public void flyWheelPIDSmartDashboard() {
    // if(((int)SmartDashboard.getNumber("Launcher Extended", 1) == 1 ? true : false) != isExtended) {
    //   if (isExtended) {
    //     retract();
    //   } else {
    //     extend();
    //   }
    // }

    // int testing = (int)SmartDashboard.getNumber("Launcher Extended", 1);
    // double testingDouble = SmartDashboard.getNumber("Launcher Extended", 1);

    // if(manualRPMSet == 1) {
    //   // read PID coefficients from SmartDashboard
    //   double p = SmartDashboard.getNumber("P Gain", 0);
    //   double i = SmartDashboard.getNumber("I Gain", 0);
    //   double d = SmartDashboard.getNumber("D Gain", 0);
    //   double iz = SmartDashboard.getNumber("I Zone", 0);
    //   double ff = SmartDashboard.getNumber("Feed Forward", 0);
    //   double max = SmartDashboard.getNumber("Max Output", 0);
    //   double min = SmartDashboard.getNumber("Min Output", 0);

    //   // if PID coefficients on SmartDashboard have changed, write new values to controller
    //   if((p != kP)) { m_flyWheelPIDController.setP(p); kP = p; }
    //   if((i != kI)) { m_flyWheelPIDController.setI(i); kI = i; }
    //   if((d != kD)) { m_flyWheelPIDController.setD(d); kD = d; }
    //   if((iz != kIz)) { m_flyWheelPIDController.setIZone(iz); kIz = iz; }
    //   if((ff != kFF)) { m_flyWheelPIDController.setFF(ff); kFF = ff; }
    //   if((max != kMaxOutput) || (min != kMinOutput)) { 
    //     m_flyWheelPIDController.setOutputRange(min, max); 
    //     kMinOutput = min; kMaxOutput = max; 
    //   }
    // }

    int inputTargetRPM = (int)SmartDashboard.getNumber("Fly Wheel Target RPM", 0);

    if (inputTargetRPM != flyWheelTargetRPM) {
      setFlyWheelRPM(inputTargetRPM);
    }

    SmartDashboard.putNumber("Fly Wheel Target RPM Check", flyWheelTargetRPM);
    
    flyWheelRPM = m_flyWheelEncoder.getVelocity();

    SmartDashboard.putNumber("Fly Wheel RPM", flyWheelRPM);
    power = SmartDashboard.getNumber("Power", power);
    // setFlyWheelPower(power);



    if (SmartDashboard.getNumber("Manual RPM set", manualRPMSet) > 0.1) {
      flyWheelTargetPower = SmartDashboard.getNumber("FlyWheel Target Power", flyWheelTargetPower);
      setFlyWheelPower(flyWheelTargetPower);
      // int inputManualRPMSet = (int)SmartDashboard.getNumber("FlyWheel Target Power", flyWheelTargetPower);
      //manualRPMSet = inputManualRPMSet;
    }

    // int inputTargetRPM = (int)SmartDashboard.getNumber("Target RPM", targetRPM);

    // double setPoint = 0;
    // if (manualRPMSet == 1) {
    //   targetRPM = inputTargetRPM;
    //   setPoint = (double)inputTargetRPM;
    // } else {
    //   setPoint = m_stick.getY()*maxRPM;
    // }

    // m_pidController.setReference(setPoint, CANSparkMax.ControlType.kVelocity);

    // // m_motor.set(m_stick.getY());
    SmartDashboard.putNumber("RPM", m_flyWheelEncoder.getVelocity());
    SmartDashboard.putNumber("PID Max", m_flyWheelPIDController.getOutputMax());
    SmartDashboard.putNumber("PID Min", m_flyWheelPIDController.getOutputMin());

    SmartDashboard.putNumber("Compressor Current", m_compressor.getCurrent());
    SmartDashboard.putBoolean("Compressor Enabled", m_compressor.isEnabled());
  }
}