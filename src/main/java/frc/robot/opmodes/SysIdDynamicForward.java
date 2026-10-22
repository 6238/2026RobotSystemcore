package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive.SysIdDirection;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Dynamic Forward", group = "Drive")
public class SysIdDynamicForward extends ScheduledOpMode {
  public SysIdDynamicForward(Robot robot) {
    super(robot.container.drive.sysIdDynamic(SysIdDirection.FORWARD));
  }
}
