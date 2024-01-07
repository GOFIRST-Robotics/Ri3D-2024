// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.commands;

import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
    private FeederSubsystem m_subsystem;

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public ClimbCommand() {
        m_subsystem = Robot.m_feedSubsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {

    }
}
