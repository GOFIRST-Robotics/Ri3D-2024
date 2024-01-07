// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimbCommand extends Command {
    private ClimberSubsystem m_subsystem;

    /** Default command */
    public ClimbCommand() {
        m_subsystem = Robot.m_climbSubsystem;
        addRequirements(m_subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        if (Robot.controller.getRawButton(Constants.UP_ARROW_AXIS)) {
            m_subsystem.setPower(Constants.CLIMBER_DEFAULT_SPEED);
        } else if (Robot.controller.getRawButton(Constants.DOWN_ARROW_AXIS)) {
            m_subsystem.setPower(-Constants.CLIMBER_DEFAULT_SPEED);
        } else {
            m_subsystem.stop();
        }
    }
}
