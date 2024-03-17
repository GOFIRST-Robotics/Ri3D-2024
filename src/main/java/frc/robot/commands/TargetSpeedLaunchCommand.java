// Author: UMN Robotics Ri3d
// Last Updated : January 2024

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.LauncherSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TargetSpeedLaunchCommand extends Command {
    private LauncherSubsystem m_launcher_subsystem;
    private FeederSubsystem m_feeder_subsystem;
    private double power, waitSec;
    private int targetRPM;
    private Timer timer = new Timer();

    /** Default feed command: runs the feed when a disc is intaked until it is ready to be shot. */
    public TargetSpeedLaunchCommand(double power, int targetRPM) {
        m_launcher_subsystem = Robot.m_launcherSubsystem;
        m_feeder_subsystem = Robot.m_feederSubsystem;
        waitSec = 0;
        this.power = power;
        this.targetRPM = targetRPM;
        addRequirements(m_launcher_subsystem, m_feeder_subsystem);
    }

    public TargetSpeedLaunchCommand(double power, int targetRPM, double waitSec) {
        m_launcher_subsystem = Robot.m_launcherSubsystem;
        m_feeder_subsystem = Robot.m_feederSubsystem;
        timer.reset();
  	    timer.start();
        this.waitSec = waitSec;
        this.power = power;
        this.targetRPM = targetRPM;
        addRequirements(m_launcher_subsystem, m_feeder_subsystem);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        m_launcher_subsystem.setFlyWheelPower(power);

        if(timer.get() >= waitSec) {
            if(m_launcher_subsystem.getFlyWheelRPM() >= targetRPM) {
                m_feeder_subsystem.setPower(-Constants.FEEDER_WHEEL_SPEED_AUTO);
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        timer.reset();
        m_launcher_subsystem.stop(); // Stop the launcher
        m_feeder_subsystem.stop(); // Stop the feeder
    }
}
