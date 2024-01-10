package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class PowerSubsystem extends SubsystemBase {

  private PowerDistribution m_revPDH;

    private double voltage, temperatureCelsius, totalCurrent, totalPower, totalEnergy, numChannels;
    private double currentsArray[]; 
    private boolean switchedChannelState;

  /** Subsystem for controlling the power of the robot */
  public PowerSubsystem() {
    m_revPDH = new PowerDistribution(1, ModuleType.kRev);

    currentsArray = new double[20];
    switchedChannelState = true;
  }

  public void setPower(double power) {} // Set the power of the robot

  @Override
    public void periodic() {
        voltage = m_revPDH.getVoltage();
        temperatureCelsius = m_revPDH.getTemperature(); // Retrieves the temperature of the PDP, in degrees Celsius.
        totalCurrent = m_revPDH.getTotalCurrent(); // Get the total current of all channels.
        totalPower = m_revPDH.getTotalPower(); // Get the total power of all channels, the bus voltage multiplied by the current with the units Watts.
        totalEnergy = m_revPDH.getTotalEnergy(); // Get the total energy of all channels with units Joules.

        // SmartDashboard.putNumber("Voltage", voltage);
        // SmartDashboard.putNumber("Total Current", totalCurrent);
        // SmartDashboard.putNumber("Total Power", totalPower);
        // SmartDashboard.putNumber("Total Energy", totalEnergy);
        // SmartDashboard.putNumber("Temperature", temperatureCelsius);
        
        numChannels = m_revPDH.getNumChannels();

        // Get the current going through all channels, in Amperes.
        // The PDP returns the current in increments of 0.125A.
        // At low currents the current readings tend to be less accurate.
        for(int i = 0; i < currentsArray.length; i++) {
            currentsArray[i] = m_revPDH.getCurrent(i);
        }
        // SmartDashboard.putNumberArray("Currents", currentsArray);
        // SmartDashboard.putNumber("Temperature", numChannels);

        // SmartDashboard.putNumber("Left Front Drive Motor Current", currentsArray[Constants.LEFT_FRONT_DRIVE_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Right Front Drive Motor Current", currentsArray[Constants.RIGHT_FRONT_DRIVE_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Left Rear Drive Motor Current", currentsArray[Constants.LEFT_REAR_DRIVE_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Right Rear Drive Motor Current", currentsArray[Constants.RIGHT_REAR_DRIVE_MOTOR_PDH_CHANNEL]);

        // SmartDashboard.putNumber("FlyWheel Motor Current", currentsArray[Constants.FLY_WHEEL_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Upper Intake Motor Current", currentsArray[Constants.UPPER_INTAKE_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Lower Intake Motor Current", currentsArray[Constants.LOWER_INTAKE_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Feeder Motor Current", currentsArray[Constants.FEEDER_WHEEL_MOTOR_PDH_CHANNEL]);
        // SmartDashboard.putNumber("Climber Drive Motor Current", currentsArray[Constants.CLIMBER_MOTOR_PDH_CHANNEL]);

        // SmartDashboard.putBoolean("Switched Channel State", switchedChannelState);
        // SmartDashboard.putBoolean("Reported Switched Channel State", getSwitchedChannelState());
    }

    public void setSwitchedChannel(boolean state) {
        switchedChannelState = state;
        m_revPDH.setSwitchableChannel(state);
    }

    public void toggleSwitchedChannel() {
        m_revPDH.setSwitchableChannel(!switchedChannelState);
        switchedChannelState = !switchedChannelState;
    }

    public boolean getSwitchedChannelState() {
        return m_revPDH.getSwitchableChannel();
    }
}
