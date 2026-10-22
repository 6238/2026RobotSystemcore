package frc.robot.opmodes;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.opmode.PeriodicOpMode;

/**
 * Runs one command while the OpMode is enabled, and cancels it when the OpMode ends. Commands v3
 * also cancels it if the OpMode is deselected, since it was scheduled inside the OpMode's scope.
 */
abstract class ScheduledOpMode extends PeriodicOpMode {
  private final Command command;

  protected ScheduledOpMode(Command command) {
    this.command = command;
  }

  @Override
  public void start() {
    Scheduler.getDefault().schedule(command);
  }

  @Override
  public void end() {
    Scheduler.getDefault().cancel(command);
  }
}
