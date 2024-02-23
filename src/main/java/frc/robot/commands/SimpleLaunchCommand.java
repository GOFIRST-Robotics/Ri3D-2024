// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SimpleLaunchCommand extends Command {
    private LauncherSubsystem m_launcher_subsystem;
    private FeederSubsystem m_feeder_subsystem;

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public SimpleLaunchCommand() {
        m_launcher_subsystem = Robot.m_launcherSubsystem;
        m_feeder_subsystem = Robot.m_feederSubsystem;
        addRequirements(m_launcher_subsystem, m_feeder_subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double targetSpeed = Constants.FLY_WHEEL_HIGH_SPEED_RPM;

        if (Robot.controller.getRawButton(Constants.RIGHT_TRIGGER_AXIS)) {
            m_launcher_subsystem.setFlyWheelPower(targetSpeed);
        }

        double currentSpeed = m_launcher_subsystem.getFlyWheelSpeed();

        if (currentSpeed == targetSpeed) {
            m_feeder_subsystem.setPower(Constants.FEEDER_WHEEL_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_launcher_subsystem.stop(); // Stop the launcher
        m_feeder_subsystem.stop(); // Stop the feeder
    }
}
