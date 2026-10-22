package frc.robot.subsystems.intake;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;

class IntakeTest {
  private Scheduler scheduler;
  private IntakeIOFake io;
  private Intake intake;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
    io = new IntakeIOFake();
    intake =
        new Intake(io) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    intake.close();
  }

  @Test
  void intakeGoesDownAndSpinsRollers() {
    io.pivotFeedback = IntakeConstants.DOWN_ANGLE;
    run(intake.intake());

    assertEquals(IntakeIOFake.Request.POSITION, io.pivotRequest);
    assertEquals(IntakeConstants.DOWN_ANGLE.in(Degrees), io.pivotPositionCommand.in(Degrees), 1e-6);
    assertEquals(IntakeIOFake.Request.VELOCITY, io.rollerRequest);
    assertEquals(
        IntakeConstants.ROLLER_SPEED.in(RotationsPerSecond),
        io.rollerVelocityCommand.in(RotationsPerSecond));
  }

  @Test
  void stowGoesUpAndStopsRollers() {
    io.pivotFeedback = IntakeConstants.UP_ANGLE;
    run(intake.stow());

    assertEquals(IntakeConstants.UP_ANGLE.in(Degrees), io.pivotPositionCommand.in(Degrees), 1e-6);
    assertEquals(IntakeIOFake.Request.VOLTAGE, io.rollerRequest);
    assertEquals(0.0, io.rollerVoltageCommand.in(Volts));
  }

  @Test
  void pivotClampNeverExceedsStartAngle() {
    io.pivotFeedback = IntakeConstants.DOWN_ANGLE;
    run(intake.ready());
    assertTrue(io.pivotPositionCommand.in(Degrees) <= IntakeConstants.START_ANGLE.in(Degrees));
    assertTrue(io.pivotPositionCommand.in(Degrees) >= IntakeConstants.DOWN_ANGLE.in(Degrees));
  }

  @Test
  void rollersStopAboveCutoff() {
    io.pivotFeedback = Degrees.of(85.0);
    intake.periodic();
    run(intake.intake());

    assertEquals(IntakeIOFake.Request.VOLTAGE, io.rollerRequest);
    assertEquals(0.0, io.rollerVoltageCommand.in(Volts));
  }

  @Test
  void oscillateStaysInsidePivotRange() {
    double down =
        IntakeConstants.DOWN_ANGLE.in(Degrees) + IntakeConstants.OSCILLATE_DOWN_OFFSET_DEGREES;
    double up = IntakeConstants.UP_ANGLE.in(Degrees);
    for (double t = 0.0; t <= 8.0; t += 0.02) {
      double setpoint = Intake.oscillateDegrees(t);
      assertTrue(setpoint >= down - 1e-6, "t=" + t + " setpoint=" + setpoint);
      assertTrue(setpoint <= up + 1e-6, "t=" + t + " setpoint=" + setpoint);
    }
  }

  private void run(Command command) {
    scheduler.schedule(command);
    scheduler.run();
  }
}
