package frc.robot.subsystems.drive;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import frc.robot.subsystems.drive.DriveFeedforwardFit.Sample;
import frc.robot.subsystems.drive.SwerveSetpointGenerator.Constraints;
import org.junit.jupiter.api.Test;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

class SwerveCalibrationTest {
  private static final double DT = 0.02;
  private static final Constraints LIMITS = new Constraints(4.0, 10.0, 2.0);

  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(
          new Translation2d(0.3, 0.3),
          new Translation2d(0.3, -0.3),
          new Translation2d(-0.3, 0.3),
          new Translation2d(-0.3, -0.3));

  @Test
  void driveSpeedStepStaysInsideAccelerationLimit() {
    SwerveModuleVelocity[] stopped = stopped();
    var setpoint =
        SwerveSetpointGenerator.generate(
            kinematics, stopped, new ChassisVelocities(4.0, 0.0, 0.0), LIMITS, DT);
    for (int i = 0; i < 4; i++) {
      assertTrue(Math.abs(setpoint.modules()[i].velocity) <= 10.0 * DT + 1e-9);
      assertTrue(Math.abs(setpoint.modules()[i].velocity) <= LIMITS.maxDriveVelocityMetersPerSec());
    }
  }

  @Test
  void azimuthStepStaysInsideSteeringLimit() {
    var setpoint =
        SwerveSetpointGenerator.generate(
            kinematics, stopped(), new ChassisVelocities(0.0, 3.0, 0.0), LIMITS, DT);
    for (SwerveModuleVelocity module : setpoint.modules()) {
      assertTrue(Math.abs(module.angle.getRadians()) <= 2.0 * DT + 1e-9);
    }
  }

  @Test
  void achievableDesiredSpeedIsUnchanged() {
    SwerveModuleVelocity[] already =
        kinematics.toSwerveModuleVelocities(new ChassisVelocities(1.0, 0.0, 0.0));
    var setpoint =
        SwerveSetpointGenerator.generate(
            kinematics, already, new ChassisVelocities(1.0, 0.0, 0.0), LIMITS, DT);
    for (int i = 0; i < 4; i++) {
      assertEquals(already[i].velocity, setpoint.modules()[i].velocity, 1e-9);
      assertEquals(0.0, setpoint.modules()[i].angle.minus(already[i].angle).getRadians(), 1e-9);
    }
  }

  @Test
  void nonFiniteDesiredKeepsPreviousFiniteState() {
    SwerveModuleVelocity[] previous = {
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO),
      new SwerveModuleVelocity(1.0, Rotation2d.ZERO)
    };
    var setpoint =
        SwerveSetpointGenerator.generate(
            kinematics, previous, new ChassisVelocities(Double.NaN, 0.0, 0.0), LIMITS, DT);
    for (int i = 0; i < 4; i++) {
      assertEquals(1.0, setpoint.modules()[i].velocity, 1e-9);
      assertTrue(Double.isFinite(setpoint.chassis().vx));
    }
  }

  @Test
  void feedforwardFitRecoversKnownGains() {
    double kS = 0.2;
    double kV = 1.5;
    double kA = 0.3;
    Sample[] samples = new Sample[6];
    double[] velocities = {1, 2, 3, 1, 2, 3};
    double[] accelerations = {0, 4, -2, 5, 1, -3};
    for (int i = 0; i < samples.length; i++) {
      double voltage = kS + kV * velocities[i] + kA * accelerations[i];
      samples[i] = new Sample(voltage, velocities[i], accelerations[i]);
    }
    DriveFeedforwardFit fit = DriveFeedforwardFit.fit(samples).orElseThrow();
    assertEquals(kS, fit.kS(), 1e-6);
    assertEquals(kV, fit.kV(), 1e-6);
    assertEquals(kA, fit.kA(), 1e-6);
  }

  @Test
  void velocityOnlyFitLeavesAccelerationGainAtZero() {
    Sample[] samples = {
      new Sample(1.7, 1.0, 50.0), new Sample(3.2, 2.0, -40.0), new Sample(4.7, 3.0, 10.0)
    };
    DriveFeedforwardFit fit = DriveFeedforwardFit.fitVelocity(samples).orElseThrow();
    assertEquals(0.2, fit.kS(), 1e-6);
    assertEquals(1.5, fit.kV(), 1e-6);
    assertEquals(0.0, fit.kA(), 0.0);
  }

  @Test
  void tooFewSamplesDoNotInventGains() {
    assertTrue(DriveFeedforwardFit.fit(new Sample[] {new Sample(1, 1, 1)}).isEmpty());
    assertTrue(
        DriveFeedforwardFit.fit(
                new Sample[] {
                  new Sample(Double.NaN, 1, 1),
                  new Sample(1, Double.NaN, 1),
                  new Sample(1, 1, Double.POSITIVE_INFINITY)
                })
            .isEmpty());
  }

  private static SwerveModuleVelocity[] stopped() {
    return new SwerveModuleVelocity[] {
      new SwerveModuleVelocity(),
      new SwerveModuleVelocity(),
      new SwerveModuleVelocity(),
      new SwerveModuleVelocity()
    };
  }
}
