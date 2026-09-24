// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import org.wpilib.opmode.Utility;

@Utility(name = "Simple FF Characterization", group = "Drive")
public class FeedforwardCharacterization extends ScheduledOpMode {
  public FeedforwardCharacterization(Robot robot) {
    super(DriveCommands.feedforwardCharacterization(robot.drive));
  }
}
