package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.*;

import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.generated.RobotTunerConstants;
import frc.robot.util.RobotIdentity;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.DoubleUnaryOperator;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.driverstation.RobotState;
import org.wpilib.hardware.hal.HAL;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Twist2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.sysid.SysIdRoutineLog;
import org.wpilib.system.Timer;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Drive implements Mechanism {
  private static final RobotTunerConstants TUNER = RobotIdentity.getTunerConstants();

  // TunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY = TUNER.kCANBus.isNetworkFD() ? 250.0 : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(TUNER.FrontLeft.LocationX, TUNER.FrontLeft.LocationY),
              Math.hypot(TUNER.FrontRight.LocationX, TUNER.FrontRight.LocationY)),
          Math.max(
              Math.hypot(TUNER.BackLeft.LocationX, TUNER.BackLeft.LocationY),
              Math.hypot(TUNER.BackRight.LocationX, TUNER.BackRight.LocationY)));

  // SysId test timing, matching the Commands v2 SysIdRoutine defaults
  private static final double SYSID_SETTLE_SECS = 1.0;
  private static final double SYSID_RAMP_VOLTS_PER_SEC = 1.0;
  private static final double SYSID_STEP_VOLTS = 7.0;
  private static final double SYSID_TIMEOUT_SECS = 10.0;

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final Alert gyroDisconnectedAlert =
      new Alert(
          "Drive",
          "gyroDisconnected",
          "Disconnected gyro, using kinematics as fallback.",
          Level.HIGH);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = Rotation2d.ZERO;
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, Pose2d.ZERO);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, TUNER.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, TUNER.FrontRight);
    modules[2] = new Module(blModuleIO, 2, TUNER.BackLeft);
    modules[3] = new Module(brModuleIO, 3, TUNER.BackRight);

    // Usage reporting for swerve template
    HAL.reportUsage("RobotDrive", "SwerveAdvantageKit");

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();
  }

  /** Updates inputs and odometry. Called by {@code Robot.robotPeriodic()} before the scheduler. */
  public void periodic() {
    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (RobotState.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    // Log empty setpoint states when disabled
    if (RobotState.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleVelocity[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleVelocity[] {});
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distance - lastModulePositions[moduleIndex].distance,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisVelocities speeds) {
    // Calculate module setpoints
    ChassisVelocities discreteSpeeds = speeds.discretize(Constants.ROBOT_PERIOD);
    SwerveModuleVelocity[] setpointStates = kinematics.toSwerveModuleVelocities(discreteSpeeds);
    setpointStates =
        SwerveDriveKinematics.desaturateWheelVelocities(setpointStates, TUNER.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      setpointStates[i] = modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(new ChassisVelocities());
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle().orElse(Rotation2d.ZERO);
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Direction of a SysId test. */
  public enum SysIdDirection {
    FORWARD,
    REVERSE
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdDirection direction) {
    double sign = direction == SysIdDirection.FORWARD ? 1.0 : -1.0;
    return sysIdTest(
        direction == SysIdDirection.FORWARD
            ? SysIdRoutineLog.State.QUASISTATIC_FORWARD
            : SysIdRoutineLog.State.QUASISTATIC_REVERSE,
        elapsedSecs -> sign * SYSID_RAMP_VOLTS_PER_SEC * elapsedSecs);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdDirection direction) {
    double sign = direction == SysIdDirection.FORWARD ? 1.0 : -1.0;
    return sysIdTest(
        direction == SysIdDirection.FORWARD
            ? SysIdRoutineLog.State.DYNAMIC_FORWARD
            : SysIdRoutineLog.State.DYNAMIC_REVERSE,
        elapsedSecs -> sign * SYSID_STEP_VOLTS);
  }

  /**
   * Orients the modules, then applies the voltage profile until the test times out or is canceled.
   */
  private Command sysIdTest(SysIdRoutineLog.State state, DoubleUnaryOperator voltageAtSecs) {
    return run(coroutine -> {
          Timer timer = new Timer();
          timer.start();
          while (timer.get() < SYSID_SETTLE_SECS) {
            runCharacterization(0.0);
            coroutine.yield();
          }

          timer.restart();
          while (timer.get() < SYSID_TIMEOUT_SECS) {
            runCharacterization(voltageAtSecs.applyAsDouble(timer.get()));
            Logger.recordOutput("Drive/SysIdState", state.toString());
            coroutine.yield();
          }
          endSysIdTest();
        })
        .whenCanceled(this::endSysIdTest)
        .named("SysId " + state);
  }

  private void endSysIdTest() {
    runCharacterization(0.0);
    Logger.recordOutput("Drive/SysIdState", SysIdRoutineLog.State.NONE.toString());
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleVelocity[] getModuleStates() {
    SwerveModuleVelocity[] states = new SwerveModuleVelocity[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  private ChassisVelocities getChassisVelocities() {
    return kinematics.toChassisVelocities(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Returns the raw gyro yaw used by teleop rotation compensation. */
  public Rotation2d getYaw() {
    return gyroInputs.yawPosition;
  }

  /** Returns the gyro yaw rate in radians per second. */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Returns whether the gyro is currently connected. */
  public boolean isGyroConnected() {
    return gyroInputs.connected;
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return TUNER.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(TUNER.FrontLeft.LocationX, TUNER.FrontLeft.LocationY),
      new Translation2d(TUNER.FrontRight.LocationX, TUNER.FrontRight.LocationY),
      new Translation2d(TUNER.BackLeft.LocationX, TUNER.BackLeft.LocationY),
      new Translation2d(TUNER.BackRight.LocationX, TUNER.BackRight.LocationY)
    };
  }
}
