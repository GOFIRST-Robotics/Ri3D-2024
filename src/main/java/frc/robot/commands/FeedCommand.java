// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import frc.robot.Constants;
import frc.robot.Robot;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FeederSubsystem;

public class FeedCommand extends Command {
  private FeederSubsystem m_feeder_subsystem;

  private boolean reverse;

  /** Creates a new IntakeCommand. */
  public FeedCommand(boolean reverse) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_feeder_subsystem = Robot.m_feederSubsystem;
    addRequirements(m_feeder_subsystem);
    this.reverse = reverse;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_feeder_subsystem.setPower(this.reverse ? -1 * Constants.FEEDER_WHEEL_SPEED : Constants.FEEDER_WHEEL_SPEED);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_feeder_subsystem.stop();
  }
}
