package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive.SysIdDirection;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Dynamic Reverse", group = "Drive")
public class SysIdDynamicReverse extends ScheduledOpMode {
  public SysIdDynamicReverse(Robot robot) {
    super(robot.container.drive.sysIdDynamic(SysIdDirection.REVERSE));
  }
}
