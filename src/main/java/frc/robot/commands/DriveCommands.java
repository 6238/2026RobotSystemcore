package frc.robot.commands;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.TeleopRotation;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.MathUtil;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

public class DriveCommands {
  private static final double DEADBAND = 0.1;
  private static final double ANGLE_KP = 5.0;
  private static final double ANGLE_KD = 0.4;
  private static final double ANGLE_MAX_VELOCITY = 8.0;
  private static final double ANGLE_MAX_ACCELERATION = 20.0;

  /** Fraction of chassis rate error added back while the rotate stick is live. */
  private static final double ROTATION_RATE_KP = 0.35;

  private static final double FF_START_DELAY = 2.0; // Secs
  private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
  private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
  private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

  private DriveCommands() {}

  private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
    double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
    Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));
    linearMagnitude = linearMagnitude * linearMagnitude;
    return new Pose2d(Translation2d.ZERO, linearDirection)
        .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.ZERO))
        .getTranslation();
  }

  /**
   * Field relative drive command using two joysticks (controlling linear and angular velocities).
   */
  public static Command joystickDrive(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier omegaSupplier) {
    return drive
        .run(
            coroutine -> {
              TeleopRotation.State rotationState = TeleopRotation.State.idle();
              while (true) {
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());

                double omegaStick = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);
                omegaStick = Math.copySign(omegaStick * omegaStick, omegaStick);
                double omegaStickRadPerSec = omegaStick * drive.getMaxAngularSpeedRadPerSec();

                TeleopRotation.Output rotation =
                    TeleopRotation.calculate(
                        omegaStickRadPerSec,
                        drive.getYaw(),
                        drive.getYawVelocityRadPerSec(),
                        drive.isGyroConnected(),
                        rotationState,
                        new TeleopRotation.Gains(
                            ROTATION_RATE_KP,
                            ANGLE_KP,
                            ANGLE_KD,
                            drive.getMaxAngularSpeedRadPerSec()));
                rotationState = rotation.next();
                Logger.recordOutput("TeleopRotation/Mode", rotation.mode().toString());
                Logger.recordOutput("TeleopRotation/OmegaStick", omegaStickRadPerSec);
                Logger.recordOutput("TeleopRotation/OmegaGyro", drive.getYawVelocityRadPerSec());
                Logger.recordOutput("TeleopRotation/OmegaOut", rotation.omegaRadPerSec());
                Logger.recordOutput("TeleopRotation/HoldHeading", rotation.next().holdHeading());
                Logger.recordOutput("TeleopRotation/Holding", rotation.next().holding());

                ChassisVelocities speeds =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        rotation.omegaRadPerSec());
                boolean isFlipped =
                    MatchState.getAlliance().isPresent()
                        && MatchState.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    speeds.toRobotRelative(
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
                coroutine.yield();
              }
            })
        .named("Joystick Drive");
  }

  /**
   * Field relative drive command using joystick for linear control and PID for angular control.
   * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
   * absolute rotation with a joystick.
   */
  public static Command joystickDriveAtAngle(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      Supplier<Rotation2d> rotationSupplier) {
    ProfiledPIDController angleController =
        new ProfiledPIDController(
            ANGLE_KP,
            0.0,
            ANGLE_KD,
            new TrapezoidProfile.Constraints(ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
    angleController.enableContinuousInput(-Math.PI, Math.PI);

    return drive
        .run(
            coroutine -> {
              angleController.reset(drive.getRotation().getRadians());
              while (true) {
                Translation2d linearVelocity =
                    getLinearVelocityFromJoysticks(
                        xSupplier.getAsDouble(), ySupplier.getAsDouble());
                double omega =
                    angleController.calculate(
                        drive.getRotation().getRadians(), rotationSupplier.get().getRadians());
                ChassisVelocities speeds =
                    new ChassisVelocities(
                        linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                        linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                        omega);
                boolean isFlipped =
                    MatchState.getAlliance().isPresent()
                        && MatchState.getAlliance().get() == Alliance.RED;
                drive.runVelocity(
                    speeds.toRobotRelative(
                        isFlipped
                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                            : drive.getRotation()));
                coroutine.yield();
              }
            })
        .named("Joystick Drive At Angle");
  }

  /**
   * Measures the velocity feedforward constants for the drive motors.
   *
   * <p>This command should only be used in voltage control mode.
   */
  public static Command feedforwardCharacterization(Drive drive) {
    List<Double> velocitySamples = new LinkedList<>();
    List<Double> voltageSamples = new LinkedList<>();
    Timer timer = new Timer();

    return drive
        .run(
            coroutine -> {
              velocitySamples.clear();
              voltageSamples.clear();

              timer.restart();
              while (timer.get() < FF_START_DELAY) {
                drive.runCharacterization(0.0);
                coroutine.yield();
              }

              timer.restart();
              while (true) {
                double voltage = timer.get() * FF_RAMP_RATE;
                drive.runCharacterization(voltage);
                velocitySamples.add(drive.getFFCharacterizationVelocity());
                voltageSamples.add(voltage);
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              int n = velocitySamples.size();
              double sumX = 0.0;
              double sumY = 0.0;
              double sumXY = 0.0;
              double sumX2 = 0.0;
              for (int i = 0; i < n; i++) {
                sumX += velocitySamples.get(i);
                sumY += voltageSamples.get(i);
                sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
              }
              double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
              double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

              NumberFormat formatter = new DecimalFormat("#0.00000");
              System.out.println("********** Drive FF Characterization Results **********");
              System.out.println("\tkS: " + formatter.format(kS));
              System.out.println("\tkV: " + formatter.format(kV));
            })
        .named("Feedforward Characterization");
  }

  /** Measures the robot's wheel radius by spinning in a circle. */
  public static Command wheelRadiusCharacterization(Drive drive) {
    SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
    WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();
    Timer timer = new Timer();

    return drive
        .run(
            coroutine -> {
              limiter.reset(0.0);
              state.measuring = false;
              timer.restart();

              while (true) {
                double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));

                if (!state.measuring) {
                  if (timer.get() >= 1.0) {
                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                    state.lastAngle = drive.getRotation();
                    state.gyroDelta = 0.0;
                    state.measuring = true;
                  }
                } else {
                  var rotation = drive.getRotation();
                  state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                  state.lastAngle = rotation;
                }
                coroutine.yield();
              }
            })
        .whenCanceled(
            () -> {
              if (!state.measuring) {
                return;
              }
              double[] positions = drive.getWheelRadiusCharacterizationPositions();
              double wheelDelta = 0.0;
              for (int i = 0; i < 4; i++) {
                wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
              }
              double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

              NumberFormat formatter = new DecimalFormat("#0.000");
              System.out.println("********** Wheel Radius Characterization Results **********");
              System.out.println("\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
              System.out.println("\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
              System.out.println(
                  "\tWheel Radius: "
                      + formatter.format(wheelRadius)
                      + " meters, "
                      + formatter.format(Units.metersToInches(wheelRadius))
                      + " inches");
            })
        .named("Wheel Radius Characterization");
  }

  private static class WheelRadiusCharacterizationState {
    double[] positions = new double[4];
    Rotation2d lastAngle = Rotation2d.ZERO;
    double gyroDelta = 0.0;
    boolean measuring = false;
  }
}
