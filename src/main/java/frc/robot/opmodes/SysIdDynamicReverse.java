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

@Utility(name = "SysId Dynamic Reverse", group = "Drive")
public class SysIdDynamicReverse extends ScheduledOpMode {
  public SysIdDynamicReverse(Robot robot) {
    super(robot.drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE));
  }
}
