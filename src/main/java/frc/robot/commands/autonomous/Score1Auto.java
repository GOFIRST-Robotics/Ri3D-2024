// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.autonomous.helperCommands.Wait;
import frc.robot.Constants;
import frc.robot.commands.TargetSpeedLaunchCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine. It drives forward for 2 seconds, turns around 180 degrees, and then drives back for 2 seconds. */
public class Score1Auto extends SequentialCommandGroup {

  // List commands here sequentially
  public Score1Auto() { // List commands here sequentially
    addCommands(new ParallelRaceGroup(new Wait(2), new TargetSpeedLaunchCommand(Constants.FLY_WHEEL_HIGH_SPEED_POWER, Constants.FLY_WHEEL_HIGH_SPEED_RPM)));
    addCommands(new TimedGyroDriveStraightCommand(1.5, -0.5));
  }
}