package frc.robot.subsystems.drive;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.util.Units;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final SwerveModuleConstants<
          TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
      constants;

  private final Alert driveDisconnectedAlert;
  private final Alert turnDisconnectedAlert;
  private final Alert turnEncoderDisconnectedAlert;
  private SwerveModulePosition[] odometryPositions = new SwerveModulePosition[] {};
  private Rotation2d lastTurnSetpoint = null;
  private double lastTurnSetpointVelRadPerSec = 0.0;

  public Module(
      ModuleIO io,
      int index,
      SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>
          constants) {
    this.io = io;
    this.index = index;
    this.constants = constants;
    driveDisconnectedAlert =
        new Alert(
            "Drive",
            "driveDisconnected" + index,
            "Disconnected drive motor on module " + index + ".",
            Level.HIGH);
    turnDisconnectedAlert =
        new Alert(
            "Drive",
            "turnDisconnected" + index,
            "Disconnected turn motor on module " + index + ".",
            Level.HIGH);
    turnEncoderDisconnectedAlert =
        new Alert(
            "Drive",
            "turnEncoderDisconnected" + index,
            "Disconnected turn encoder on module " + index + ".",
            Level.HIGH);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // Calculate positions for odometry
    int sampleCount = inputs.odometryTimestamps.length; // All signals are sampled together
    odometryPositions = new SwerveModulePosition[sampleCount];
    for (int i = 0; i < sampleCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * constants.WheelRadius;
      Rotation2d angle = inputs.odometryTurnPositions[i];
      odometryPositions[i] = new SwerveModulePosition(positionMeters, angle);
    }

    // Update alerts
    driveDisconnectedAlert.set(!inputs.driveConnected);
    turnDisconnectedAlert.set(!inputs.turnConnected);
    turnEncoderDisconnectedAlert.set(!inputs.turnEncoderConnected);
  }

  /** Runs the module with the specified setpoint state. Returns the optimized setpoint. */
  public SwerveModuleVelocity runSetpoint(SwerveModuleVelocity state) {
    SwerveModuleVelocity optimized = state.optimize(getAngle()).cosineScale(inputs.turnPosition);
    double turnVelocityRadPerSec =
        setpointVelocityRadPerSec(lastTurnSetpoint, optimized.angle, Constants.ROBOT_PERIOD);
    lastTurnSetpoint = optimized.angle;
    io.setDriveVelocity(optimized.velocity / constants.WheelRadius);
    io.setTurnPosition(optimized.angle, turnVelocityRadPerSec);
    lastTurnSetpointVelRadPerSec = turnVelocityRadPerSec;
    return optimized;
  }

  /**
   * Steer rate implied by consecutive azimuth setpoints. A ≥90° jump is an optimize flip, not a
   * slew — velocity feedforward must stay 0.
   */
  static double setpointVelocityRadPerSec(
      Rotation2d lastSetpoint, Rotation2d nextSetpoint, double dt) {
    if (lastSetpoint == null
        || nextSetpoint == null
        || !Double.isFinite(dt)
        || dt <= 0.0
        || !Double.isFinite(lastSetpoint.getRadians())
        || !Double.isFinite(nextSetpoint.getRadians())) {
      return 0.0;
    }
    double dTheta = nextSetpoint.minus(lastSetpoint).getRadians();
    if (!Double.isFinite(dTheta) || Math.abs(dTheta) > Math.PI / 2.0) {
      return 0.0;
    }
    double velocity = dTheta / dt;
    return Double.isFinite(velocity) ? velocity : 0.0;
  }

  /** Runs the module with the specified output while controlling to zero degrees. */
  public void runCharacterization(double output) {
    io.setDriveOpenLoop(output);
    io.setTurnPosition(Rotation2d.ZERO);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setDriveOpenLoop(0.0);
    io.setTurnOpenLoop(0.0);
  }

  public double getTurnSetpointVelRadPerSec() {
    return lastTurnSetpointVelRadPerSec;
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionRad * constants.WheelRadius;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityRadPerSec * constants.WheelRadius;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleVelocity getState() {
    return new SwerveModuleVelocity(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module positions received this cycle. */
  public SwerveModulePosition[] getOdometryPositions() {
    return odometryPositions;
  }

  /** Returns the timestamps of the samples received this cycle. */
  public double[] getOdometryTimestamps() {
    return inputs.odometryTimestamps;
  }

  /** Returns the module position in radians. */
  public double getWheelRadiusCharacterizationPosition() {
    return inputs.drivePositionRad;
  }

  /** Returns the module velocity in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    return Units.radiansToRotations(inputs.driveVelocityRadPerSec);
  }
}
