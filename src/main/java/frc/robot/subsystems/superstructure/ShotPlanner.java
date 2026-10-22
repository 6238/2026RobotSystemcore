package frc.robot.subsystems.superstructure;

import static org.wpilib.units.Units.RotationsPerSecond;

import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.util.ExtrapolatingDoubleTreeMap;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.measure.AngularVelocity;

/** Lead-time shot and pass setpoints. Pure functions of pose, speeds, and the shooter maps. */
public final class ShotPlanner {
  private ShotPlanner() {}

  public static ShotSetpoint createShotSetpoint(
      Pose2d hub, Pose2d drivePose, ChassisVelocities robotSpeeds) {
    return plan(
        drivePose,
        robotSpeeds,
        hub.getTranslation(),
        ShooterConstants.LEAD_TIME_MAP,
        ShooterConstants.FLYWHEEL_MAP,
        Double.POSITIVE_INFINITY);
  }

  public static ShotSetpoint createPassSetpoint(
      Pose2d targetPassPoint, Pose2d drivePose, ChassisVelocities robotSpeeds) {
    return plan(
        drivePose,
        robotSpeeds,
        targetPassPoint.getTranslation(),
        ShooterConstants.PASSING_LEAD_TIME_MAP,
        ShooterConstants.PASSING_FLYWHEEL_MAP,
        ShooterConstants.PASSING_FLYWHEEL_MAX_RPS);
  }

  private static ShotSetpoint plan(
      Pose2d drivePose,
      ChassisVelocities robotSpeeds,
      Translation2d target,
      ExtrapolatingDoubleTreeMap leadTimeMap,
      ExtrapolatingDoubleTreeMap flywheelMap,
      double maxFlywheelRps) {
    ChassisVelocities fieldSpeeds = robotSpeeds.toFieldRelative(drivePose.getRotation());
    Translation2d robotTranslation = drivePose.getTranslation();

    double leadTimeSec = leadTimeMap.get(robotTranslation.getDistance(target));
    Translation2d virtualTarget = target;
    for (int i = 0; i < 3; i++) {
      virtualTarget =
          target.plus(
              new Translation2d(fieldSpeeds.vx * -leadTimeSec, fieldSpeeds.vy * -leadTimeSec));
      leadTimeSec = leadTimeMap.get(robotTranslation.getDistance(virtualTarget));
    }

    double predictedDistanceM = robotTranslation.getDistance(virtualTarget);
    double flywheelSpeed = Math.min(flywheelMap.get(predictedDistanceM), maxFlywheelRps);
    double feederSpeed = ShooterConstants.FEEDER_MAP.get(predictedDistanceM);
    Rotation2d heading = virtualTarget.minus(robotTranslation).getAngle().orElse(Rotation2d.ZERO);

    return new ShotSetpoint(
        RotationsPerSecond.of(flywheelSpeed),
        RotationsPerSecond.of(feederSpeed),
        new Pose2d(robotTranslation, heading),
        virtualTarget,
        robotSpeeds);
  }

  public record ShotSetpoint(
      AngularVelocity flywheelSpeed,
      AngularVelocity feederSpeed,
      Pose2d robotPose,
      Translation2d target,
      ChassisVelocities robotSpeeds) {
    public ShotSetpoint() {
      this(
          RotationsPerSecond.of(0.0),
          RotationsPerSecond.of(0.0),
          Pose2d.ZERO,
          Translation2d.ZERO,
          new ChassisVelocities());
    }
  }
}
