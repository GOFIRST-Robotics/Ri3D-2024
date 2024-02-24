// Author: UMN Robotics Ri3d
// Last Updated : January 2023

package frc.robot.commands.autonomous;

import frc.robot.commands.TimedGyroDriveStraightCommand;
import frc.robot.commands.autonomous.helperCommands.Wait;
import frc.robot.Constants;
import frc.robot.commands.FeedCommand;
import frc.robot.commands.LimitSwitchIntakeCommand;
import frc.robot.commands.SimpleLaunchCommand;
import frc.robot.commands.TargetSpeedLaunchCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Autonomous Mode (Default) ******************************************************
 * This is our most basic autonomous routine. It drives forward for 2 seconds, turns around 180 degrees, and then drives back for 2 seconds. */
public class Score2Auto extends SequentialCommandGroup {

  // List commands here sequentially
  public Score2Auto() { // List commands here sequentially
    addCommands(
      new ParallelRaceGroup(
        new Wait(2), 
        new TargetSpeedLaunchCommand(Constants.FLY_WHEEL_HIGH_SPEED_POWER, Constants.FLY_WHEEL_HIGH_SPEED_RPM)
      )
    );
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new TimedGyroDriveStraightCommand(1.5, -0.4),
          new TimedGyroDriveStraightCommand(1.5, -0.2),
          new Wait(0.5)
        ),
        new SequentialCommandGroup(
          new Wait(1), 
          new ParallelCommandGroup( 
            new LimitSwitchIntakeCommand(true),
            new SimpleLaunchCommand(-0.1)
          )
        )
      )
    );
    addCommands(
      new ParallelDeadlineGroup(
        new SequentialCommandGroup(
          new TimedGyroDriveStraightCommand(1.5, 0.4),
          new TimedGyroDriveStraightCommand(0.75, 0.2),
          new Wait(4)
        ),
        new SequentialCommandGroup(
          new Wait(1.75), 
          new ParallelDeadlineGroup(
            new Wait(0.2), 
            new FeedCommand(false)
          ),
          new TargetSpeedLaunchCommand(Constants.FLY_WHEEL_HIGH_SPEED_POWER, Constants.FLY_WHEEL_HIGH_SPEED_RPM)
        )
      )
    );

    addCommands(new TimedGyroDriveStraightCommand(2, -0.4));
  }
}