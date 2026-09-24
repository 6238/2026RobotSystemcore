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
