package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.FeedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class FeedCommand extends Command {
    private FeedSubsystem m_subsystem;

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public FeedCommand() {
        m_subsystem = Robot.m_feedSubsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {

    }
}
