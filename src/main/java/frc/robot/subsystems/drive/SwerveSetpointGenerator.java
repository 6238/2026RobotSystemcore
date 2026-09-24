package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

/**
 * Limits a desired chassis velocity to module speeds and azimuths the modules can reach in one
 * loop. The Talon velocity loop then tracks that step with kV and kA; this class is the profile.
 */
public final class SwerveSetpointGenerator {
  /** Drive and azimuth limits enforced on every generated step. */
  public record Constraints(
      double maxDriveVelocityMetersPerSec,
      double maxDriveAccelerationMetersPerSecSq,
      double maxSteeringVelocityRadPerSec) {}

  /** Chassis velocity that the limited module states actually produce. */
  public record Setpoint(ChassisVelocities chassis, SwerveModuleVelocity[] modules) {}

  private SwerveSetpointGenerator() {}

  /**
   * @param previous module states commanded last cycle. Null or empty starts from stopped modules
   *     at angle zero.
   */
  public static Setpoint generate(
      SwerveDriveKinematics kinematics,
      SwerveModuleVelocity[] previous,
      ChassisVelocities desired,
      Constraints constraints,
      double dtSeconds) {
    SwerveModuleVelocity[] last = copyOrStopped(previous);
    if (!(dtSeconds > 0.0) || !finite(desired)) {
      return new Setpoint(kinematics.toChassisVelocities(last), last);
    }

    SwerveModuleVelocity[] desiredModules =
        kinematics.toSwerveModuleVelocities(
            new ChassisVelocities(desired.vx, desired.vy, desired.omega));
    desiredModules =
        SwerveDriveKinematics.desaturateWheelVelocities(
            desiredModules, constraints.maxDriveVelocityMetersPerSec);

    SwerveModuleVelocity[] limited = new SwerveModuleVelocity[last.length];
    for (int i = 0; i < last.length; i++) {
      limited[i] = limitModule(last[i], desiredModules[i], constraints, dtSeconds);
    }
    return new Setpoint(kinematics.toChassisVelocities(limited), limited);
  }

  private static SwerveModuleVelocity limitModule(
      SwerveModuleVelocity previous,
      SwerveModuleVelocity desired,
      Constraints constraints,
      double dtSeconds) {
    if (!finite(desired)) {
      return previous;
    }
    SwerveModuleVelocity optimized = desired.optimize(previous.angle);
    double angleError = optimized.angle.minus(previous.angle).getRadians();
    double maxAngleStep = constraints.maxSteeringVelocityRadPerSec * dtSeconds;
    double angleStep = Math.clamp(angleError, -maxAngleStep, maxAngleStep);
    Rotation2d achievableAngle = previous.angle.plus(new Rotation2d(angleStep));

    double speedTarget = optimized.velocity;
    if (Math.abs(angleError) > maxAngleStep) {
      double heading = achievableAngle.getRadians();
      speedTarget = optimized.velocity * Math.cos(optimized.angle.getRadians() - heading);
    }
    double maxSpeedStep = constraints.maxDriveAccelerationMetersPerSecSq * dtSeconds;
    double speed =
        previous.velocity
            + Math.clamp(speedTarget - previous.velocity, -maxSpeedStep, maxSpeedStep);
    speed =
        Math.clamp(
            speed,
            -constraints.maxDriveVelocityMetersPerSec,
            constraints.maxDriveVelocityMetersPerSec);
    if (!Double.isFinite(speed)) {
      speed = 0.0;
    }
    return new SwerveModuleVelocity(speed, achievableAngle);
  }

  private static SwerveModuleVelocity[] copyOrStopped(SwerveModuleVelocity[] previous) {
    if (previous == null || previous.length == 0) {
      return new SwerveModuleVelocity[] {
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity(),
        new SwerveModuleVelocity()
      };
    }
    SwerveModuleVelocity[] copy = new SwerveModuleVelocity[previous.length];
    for (int i = 0; i < previous.length; i++) {
      SwerveModuleVelocity state = previous[i];
      copy[i] =
          state == null || !finite(state)
              ? new SwerveModuleVelocity()
              : new SwerveModuleVelocity(state.velocity, state.angle);
    }
    return copy;
  }

  private static boolean finite(ChassisVelocities speeds) {
    return speeds != null
        && Double.isFinite(speeds.vx)
        && Double.isFinite(speeds.vy)
        && Double.isFinite(speeds.omega);
  }

  private static boolean finite(SwerveModuleVelocity state) {
    return state != null
        && state.angle != null
        && Double.isFinite(state.velocity)
        && Double.isFinite(state.angle.getRadians());
  }
}
