package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.superstructure.ShotPlanner.ShotSetpoint;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.units.Units;

class ShotPlannerTest {
  @Test
  void stationaryShotLooksAtHubAndStaysFinite() {
    Pose2d hub = new Pose2d(4.625, 4.045, Rotation2d.ZERO);
    Pose2d robot = new Pose2d(2.0, 4.045, Rotation2d.ZERO);

    ShotSetpoint setpoint = ShotPlanner.createShotSetpoint(hub, robot, new ChassisVelocities());

    assertTrue(Double.isFinite(setpoint.flywheelSpeed().in(Units.RotationsPerSecond)));
    assertTrue(Double.isFinite(setpoint.feederSpeed().in(Units.RotationsPerSecond)));
    assertTrue(setpoint.flywheelSpeed().in(Units.RotationsPerSecond) > 0.0);

    double headingErrorDeg =
        Math.abs(setpoint.robotPose().getRotation().minus(new Rotation2d(0.0)).getDegrees());
    assertTrue(headingErrorDeg < 1.0, "heading error " + headingErrorDeg);
  }

  @Test
  void movingRobotShiftsVirtualTargetOppositeVelocity() {
    Pose2d hub = new Pose2d(4.625, 4.045, Rotation2d.ZERO);
    Pose2d robot = new Pose2d(2.0, 4.045, Rotation2d.ZERO);
    ChassisVelocities towardHub = new ChassisVelocities(2.0, 0.0, 0.0);

    ShotSetpoint moving = ShotPlanner.createShotSetpoint(hub, robot, towardHub);
    ShotSetpoint still = ShotPlanner.createShotSetpoint(hub, robot, new ChassisVelocities());

    assertTrue(moving.target().getX() < still.target().getX());
  }
}
