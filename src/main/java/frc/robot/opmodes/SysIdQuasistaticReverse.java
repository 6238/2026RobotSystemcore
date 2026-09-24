package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Quasistatic Reverse", group = "Drive")
public class SysIdQuasistaticReverse extends ScheduledOpMode {
  public SysIdQuasistaticReverse(Robot robot) {
    super(robot.drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
  }
}
