package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.subsystems.drive.Drive.SysIdDirection;
import org.wpilib.opmode.Utility;

@Utility(name = "SysId Quasistatic Forward", group = "Drive")
public class SysIdQuasistaticForward extends ScheduledOpMode {
  public SysIdQuasistaticForward(Robot robot) {
    super(robot.container.drive.sysIdQuasistatic(SysIdDirection.FORWARD));
  }
}
