package frc.robot;

import static frc.robot.subsystems.vision.VisionConstants.*;

import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOTalonFX;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.vision.VisionIOPhotonVisionSim;
import frc.robot.util.RobotIdentity;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.button.CommandXboxController;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.hardware.power.PowerDistribution;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * Owns the robot's mechanisms, controllers, and global button bindings. OpModes reach these through
 * {@link Robot#container}.
 */
public class RobotContainer {
  public final Drive drive;
  public final Vision vision;
  public final Shooter shooter;
  public final CommandXboxController controller = new CommandXboxController(0);
  private PowerDistribution pdh;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        pdh = new PowerDistribution(CANPort.CAN_S1, 9, PowerDistribution.ModuleType.REV);
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOTalonFX(RobotIdentity.getTunerConstants().BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVision(camera0Name, robotToCamera0),
                new VisionIOPhotonVision(camera1Name, robotToCamera1));
        shooter = new Shooter(new ShooterIOTalonFX());
        break;

      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIOSim(RobotIdentity.getTunerConstants().FrontLeft),
                new ModuleIOSim(RobotIdentity.getTunerConstants().FrontRight),
                new ModuleIOSim(RobotIdentity.getTunerConstants().BackLeft),
                new ModuleIOSim(RobotIdentity.getTunerConstants().BackRight));
        vision =
            new Vision(
                drive::addVisionMeasurement,
                drive::getPose,
                new VisionIOPhotonVisionSim(camera0Name, robotToCamera0, drive::getPose),
                new VisionIOPhotonVisionSim(camera1Name, robotToCamera1, drive::getPose));
        shooter = new Shooter(new ShooterIO() {});
        break;

      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});
        vision =
            new Vision(
                drive::addVisionMeasurement, drive::getPose, new VisionIO() {}, new VisionIO() {});
        shooter = new Shooter(new ShooterIO() {});
        break;
    }

    configureButtonBindings();
  }

  /**
   * Bindings made here, before any OpMode is selected, are global. Bindings made inside an OpMode
   * only live as long as that OpMode.
   */
  private void configureButtonBindings() {
    // Lock to 0° when A button is held
    controller
        .a()
        .whileTrue(
            DriveCommands.joystickDriveAtAngle(
                drive,
                () -> -controller.getController().getLeftY(),
                () -> -controller.getController().getLeftX(),
                () -> Rotation2d.ZERO));

    // Switch to X pattern when X button is pressed
    controller.x().onTrue(drive.run(coroutine -> drive.stopWithX()).named("Stop With X"));

    // Reset gyro to 0° when B button is pressed
    controller
        .b()
        .onTrue(
            drive
                .run(
                    coroutine ->
                        drive.setPose(
                            new Pose2d(drive.getPose().getTranslation(), Rotation2d.ZERO)))
                .named("Reset Heading"));
  }

  /** Logs robot-level telemetry that does not belong to a mechanism. */
  public void periodic() {
    if (pdh != null) {
      Logger.recordOutput("PDH/Voltage", pdh.getVoltage());
      Logger.recordOutput("PDH/TotalCurrent", pdh.getTotalCurrent());
    }
  }
}
