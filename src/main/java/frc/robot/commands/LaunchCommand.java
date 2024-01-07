package frc.robot.commands;

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
    public void initialize() {

    }
}
