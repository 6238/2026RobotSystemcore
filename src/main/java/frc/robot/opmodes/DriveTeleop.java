package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.DriveCommands;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Field-oriented swerve drive. Button bindings live in {@link RobotContainer}. The joystick default
 * command is set inside this OpMode, so Commands v3 scopes it to the OpMode and restores the
 * drive's previous default command (idle) once a different OpMode is selected.
 */
@Teleop(name = "Drive")
public class DriveTeleop extends PeriodicOpMode {
  private final RobotContainer container;
  private boolean defaultCommandSet;

  public DriveTeleop(Robot robot) {
    this.container = robot.container;
  }

  @Override
  public void start() {
    // The binding stays active across disable/enable cycles of this OpMode; only add it once
    if (defaultCommandSet) {
      return;
    }
    defaultCommandSet = true;
    container.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            container.drive,
            () -> -container.controller.getController().getLeftY(),
            () -> -container.controller.getController().getLeftX(),
            () -> -container.controller.getController().getRightX()));
  }
}
