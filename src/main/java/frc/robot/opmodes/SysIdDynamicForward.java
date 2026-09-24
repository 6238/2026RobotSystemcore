package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Dynamic Forward", group = "Drive")
public class SysIdDynamicForward extends ScheduledOpMode {
  public SysIdDynamicForward(Robot robot) {
    super(robot.drive.sysIdDynamic(SysIdRoutine.Direction.FORWARD));
  }
}
