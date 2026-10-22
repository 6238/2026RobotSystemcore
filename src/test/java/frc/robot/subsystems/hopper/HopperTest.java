package frc.robot.subsystems.hopper;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.units.measure.AngularVelocity;

class HopperTest {
  private static final AngularVelocity TOLERANCE = RotationsPerSecond.of(2.0);

  private Scheduler scheduler;
  private HopperIOFake io;
  private Hopper hopper;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
    io = new HopperIOFake();
    hopper =
        new Hopper(io) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    hopper.close();
  }

  @Test
  void feedWritesBothVelocities() {
    run(hopper.feed(RotationsPerSecond.of(40.0), RotationsPerSecond.of(50.0)));

    assertEquals(HopperIOFake.Request.VELOCITY, io.indexerRequest);
    assertEquals(40.0, io.indexerVelocityCommand.in(RotationsPerSecond));
    assertEquals(HopperIOFake.Request.VELOCITY, io.topIndexerRequest);
    assertEquals(50.0, io.topIndexerVelocityCommand.in(RotationsPerSecond));
  }

  @Test
  void nonFiniteIndexerIdlesThatActuator() {
    run(hopper.feed(RotationsPerSecond.of(Double.NaN), RotationsPerSecond.of(40.0)));

    assertEquals(HopperIOFake.Request.VOLTAGE, io.indexerRequest);
    assertEquals(0.0, io.indexerVoltageCommand.in(Volts));
    assertEquals(HopperIOFake.Request.VELOCITY, io.topIndexerRequest);
    assertFalse(hopper.indexerAtSpeed(TOLERANCE));
  }

  @Test
  void reverseWritesNegativeVelocities() {
    run(hopper.reverse());

    assertEquals(
        -HopperConstants.INDEXER_SPEED.in(RotationsPerSecond),
        io.indexerVelocityCommand.in(RotationsPerSecond));
    assertEquals(
        -HopperConstants.TOP_INDEXER_SPEED.in(RotationsPerSecond),
        io.topIndexerVelocityCommand.in(RotationsPerSecond));
  }

  @Test
  void idleZerosBoth() {
    run(hopper.feed());
    scheduler.cancelAll();
    scheduler.run();

    assertEquals(HopperIOFake.Request.VOLTAGE, io.indexerRequest);
    assertEquals(0.0, io.indexerVoltageCommand.in(Volts));
    assertEquals(HopperIOFake.Request.VOLTAGE, io.topIndexerRequest);
    assertEquals(0.0, io.topIndexerVoltageCommand.in(Volts));
  }

  private void run(Command command) {
    scheduler.schedule(command);
    scheduler.run();
  }
}
