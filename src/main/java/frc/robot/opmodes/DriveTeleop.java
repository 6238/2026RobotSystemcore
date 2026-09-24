package frc.robot.opmodes;

import frc.robot.Robot;
import frc.robot.commands.DriveCommands;
import org.wpilib.command2.Commands;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.opmode.PeriodicOpMode;
import org.wpilib.opmode.Teleop;

/**
 * Field-oriented swerve drive. Button bindings are installed once; the OpMode is reconstructed
 * every time it is selected, and trigger bindings stay registered on the command scheduler.
 */
@Teleop(name = "Drive")
public class DriveTeleop extends PeriodicOpMode {
  private static boolean bindingsConfigured;

  private final Robot robot;

  public DriveTeleop(Robot robot) {
    this.robot = robot;
    if (!bindingsConfigured) {
      configureBindings();
      bindingsConfigured = true;
    }
  }

  @Override
  public void start() {
    robot.drive.setDefaultCommand(
        DriveCommands.joystickDrive(
            robot.drive,
            () -> -robot.controller.getController().getLeftY(),
            () -> -robot.controller.getController().getLeftX(),
            () -> -robot.controller.getController().getRightX()));
  }

  @Override
  public void end() {
    robot.drive.removeDefaultCommand();
  }

  private void configureBindings() {
    // Lock to 0° when A button is held
    robot
        .controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                robot.drive,
                () -> -robot.controller.getController().getLeftY(),
                () -> -robot.controller.getController().getLeftX(),
                () -> Rotation2d.ZERO));

    // Switch to X pattern when X button is pressed
    robot.controller.x().onTrue(Commands.runOnce(robot.drive::stopWithX, robot.drive));

    // Reset gyro to 0° when B button is pressed
    robot
        .controller
        .b()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robot.drive.setPose(
                            new Pose2d(robot.drive.getPose().getTranslation(), Rotation2d.ZERO)),
                    robot.drive)
                .ignoringDisable(true));
  }
}
