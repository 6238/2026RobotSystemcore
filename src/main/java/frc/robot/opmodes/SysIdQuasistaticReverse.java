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

@Utility(name = "SysId Quasistatic Reverse", group = "Drive")
public class SysIdQuasistaticReverse extends ScheduledOpMode {
  public SysIdQuasistaticReverse(Robot robot) {
    super(robot.drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE));
  }
}
