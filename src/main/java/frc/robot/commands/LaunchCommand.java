// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class LaunchCommand extends Command {
    private LauncherSubsystem m_subsystem;

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public LaunchCommand() {
        m_subsystem = Robot.m_launcherSubsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        double targetSpeed = Constants.FLY_WHEEL_DEFAULT_SPEED;

        if (Robot.controller.getRawButton(Constants.RIGHT_TRIGGER_AXIS)) {
            m_subsystem.flyWheelPower(targetSpeed);
        }

        double currentSpeed = m_subsystem.getFlyWheelSpeed();

        if (currentSpeed == targetSpeed) {
            m_subsystem.feederWheelPower(Constants.FEEDER_WHEEL_DEFAULT_SPEED);
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_subsystem.stop(); // Stop the flywheel and feeder wheel motors
    }
}
