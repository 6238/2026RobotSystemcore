package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive.SysIdDirection;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Quasistatic Reverse", group = "Drive")
public class SysIdQuasistaticReverse extends ScheduledOpMode {
  public SysIdQuasistaticReverse(Robot robot) {
    super(robot.container.drive.sysIdQuasistatic(SysIdDirection.REVERSE));
  }
}
