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
    private double power;

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public SimpleLaunchCommand(double power) {
        m_launcher_subsystem = Robot.m_launcherSubsystem;
        m_feeder_subsystem = Robot.m_feederSubsystem;
        this.power = power;
        addRequirements(m_launcher_subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_launcher_subsystem.setFlyWheelPower(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_launcher_subsystem.stop(); // Stop the launcher
        m_feeder_subsystem.stop(); // Stop the feeder
    }
}
