package frc.robot.subsystems.shooter;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.AngularVelocity;

class ShooterTest {
  private static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2.0);

  private Scheduler scheduler;
  private ShooterIOFake io;
  private Shooter shooter;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
    io = new ShooterIOFake();
    shooter =
        new Shooter(io) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    shooter.close();
  }

  @Test
  void spinUpReversesFeeder() {
    run(shooter.spinUp(RotationsPerSecond.of(40.0)));

    assertEquals(ShooterIOFake.Request.VELOCITY, io.flywheelRequest);
    assertEquals(40.0, io.flywheelVelocityCommand.in(RotationsPerSecond));
    assertEquals(ShooterIOFake.Request.VOLTAGE, io.feederRequest);
    assertEquals(-ShooterConstants.FEEDER_REVERSE_VOLTS, io.feederVoltageCommand.in(Volts));
  }

  @Test
  void nonFiniteFlywheelVelocityIdlesFlywheel() {
    run(shooter.holdFlywheel(RotationsPerSecond.of(Double.NaN)));

    assertEquals(ShooterIOFake.Request.VOLTAGE, io.flywheelRequest);
    assertEquals(0.0, io.flywheelVoltageCommand.in(Volts));
    assertFalse(shooter.flywheelAtSpeed(TOLERANCE));
  }

  @Test
  void idleClearsReady() {
    run(shooter.holdFlywheel(RotationsPerSecond.of(50.0)));
    io.flywheelFeedback = RotationsPerSecond.of(50.0);
    shooter.periodic();
    assertTrue(shooter.flywheelAtSpeed(TOLERANCE));

    scheduler.cancelAll();
    scheduler.run();
    shooter.periodic();
    assertFalse(shooter.flywheelAtSpeed(TOLERANCE));
  }

  @Test
  void beamBreakOpensShootingWindow() {
    assertFalse(shooter.isShooting());
    io.beamBreakTriggered = true;
    shooter.periodic();
    assertTrue(shooter.isShooting());
  }

  private void run(Command command) {
    scheduler.schedule(command);
    scheduler.run();
  }
}
