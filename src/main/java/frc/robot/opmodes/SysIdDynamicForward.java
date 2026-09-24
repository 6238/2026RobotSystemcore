// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

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
