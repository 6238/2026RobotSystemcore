package frc.robot.subsystems.drive;

import org.wpilib.math.geometry.Rotation2d;

/**
 * Teleop chassis rotation: rate loop while the rotate stick is live, heading lock once it returns
 * to deadband. No ghost pose. Gyro missing → stick passthrough.
 */
public final class TeleopRotation {
  public enum Mode {
    /** Rotate stick live: stick feedforward plus gyro rate correction. */
    RATE,
    /** Rotate stick in deadband: hold the heading snapshotted on release. */
    HOLD,
    /** No usable gyro: command the stick and forget hold state. */
    OPEN
  }

  public record State(boolean holding, Rotation2d holdHeading) {
    public static State idle() {
      return new State(false, Rotation2d.ZERO);
    }
  }

  public record Output(double omegaRadPerSec, State next, Mode mode) {}

  public record Gains(double kRate, double holdKp, double holdKd, double maxOmegaRadPerSec) {}

  private TeleopRotation() {}

  /**
   * {@code omegaStickRadPerSec} is already deadbanded: zero means hold, nonzero means rate. Output
   * is always finite and within {@code maxOmegaRadPerSec}.
   */
  public static Output calculate(
      double omegaStickRadPerSec,
      Rotation2d heading,
      double gyroOmegaRadPerSec,
      boolean gyroConnected,
      State previous,
      Gains gains) {
    if (previous == null || gains == null || !usableHeading(heading)) {
      return new Output(0.0, State.idle(), Mode.OPEN);
    }

    double maxOmega = finiteAbs(gains.maxOmegaRadPerSec());
    if (!Double.isFinite(omegaStickRadPerSec) || !Double.isFinite(gyroOmegaRadPerSec)) {
      return new Output(0.0, State.idle(), Mode.OPEN);
    }

    if (!gyroConnected) {
      return new Output(clamp(omegaStickRadPerSec, maxOmega), State.idle(), Mode.OPEN);
    }

    if (omegaStickRadPerSec != 0.0) {
      double omega =
          omegaStickRadPerSec + gains.kRate() * (omegaStickRadPerSec - gyroOmegaRadPerSec);
      return new Output(clamp(omega, maxOmega), State.idle(), Mode.RATE);
    }

    Rotation2d holdHeading = previous.holding() ? previous.holdHeading() : heading;
    if (!usableHeading(holdHeading)) {
      holdHeading = heading;
    }
    double headingError = holdHeading.minus(heading).getRadians();
    double omega = gains.holdKp() * headingError - gains.holdKd() * gyroOmegaRadPerSec;
    return new Output(clamp(omega, maxOmega), new State(true, holdHeading), Mode.HOLD);
  }

  private static boolean usableHeading(Rotation2d heading) {
    return heading != null && Double.isFinite(heading.getRadians());
  }

  private static double finiteAbs(double value) {
    return Double.isFinite(value) ? Math.abs(value) : 0.0;
  }

  private static double clamp(double value, double maxAbs) {
    if (!Double.isFinite(value)) {
      return 0.0;
    }
    return Math.clamp(value, -maxAbs, maxAbs);
  }
}
