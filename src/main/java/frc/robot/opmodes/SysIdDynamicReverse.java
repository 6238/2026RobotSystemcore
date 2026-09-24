package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Dynamic Reverse", group = "Drive")
public class SysIdDynamicReverse extends ScheduledOpMode {
  public SysIdDynamicReverse(Robot robot) {
    super(robot.drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
  }
}
