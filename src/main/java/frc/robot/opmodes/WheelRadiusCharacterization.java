package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import org.wpilib.opmode.Utility;

@Utility(name = "Wheel Radius Characterization", group = "Drive")
public class WheelRadiusCharacterization extends ScheduledOpMode {
  public WheelRadiusCharacterization(Robot robot) {
    super(DriveCommands.wheelRadiusCharacterization(robot.container.drive));
  }
}
