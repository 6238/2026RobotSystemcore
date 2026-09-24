// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.opmodes;

import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.opmode.PeriodicOpMode;

/** Runs one command while the OpMode is enabled, and cancels it when the OpMode ends. */
abstract class ScheduledOpMode extends PeriodicOpMode {
  private final Command command;

  protected ScheduledOpMode(Command command) {
    this.command = command;
  }

  @Override
  public void start() {
    CommandScheduler.getInstance().schedule(command);
  }

  @Override
  public void end() {
    command.cancel();
  }
}
