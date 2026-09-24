package frc.robot.opmodes;

import frc.robot.Robot;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Quasistatic Forward", group = "Drive")
public class SysIdQuasistaticForward extends ScheduledOpMode {
  public SysIdQuasistaticForward(Robot robot) {
    super(robot.drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD));
  }
}
