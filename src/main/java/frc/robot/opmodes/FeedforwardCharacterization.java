package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import org.wpilib.opmode.Utility;

@Utility(name = "Simple FF Characterization", group = "Drive")
public class FeedforwardCharacterization extends ScheduledOpMode {
  public FeedforwardCharacterization(Robot robot) {
    super(DriveCommands.feedforwardCharacterization(robot.drive));
  }
}
