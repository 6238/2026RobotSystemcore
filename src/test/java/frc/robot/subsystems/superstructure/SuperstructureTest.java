package frc.robot.subsystems.superstructure;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.wpilib.units.Units.RotationsPerSecond;

import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperIOFake;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIOFake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIOFake;
import frc.robot.subsystems.superstructure.Superstructure.Current;
import frc.robot.subsystems.superstructure.Superstructure.Inputs;
import frc.robot.subsystems.superstructure.Superstructure.Wanted;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;

class SuperstructureTest {
  private Scheduler scheduler;
  private ShooterIOFake shooterIo;
  private HopperIOFake hopperIo;
  private IntakeIOFake intakeIo;
  private Shooter shooter;
  private Hopper hopper;
  private Intake intake;
  private Superstructure superstructure;

  @BeforeEach
  void setUp() {
    scheduler = Scheduler.createIndependentScheduler();
    shooterIo = new ShooterIOFake();
    hopperIo = new HopperIOFake();
    intakeIo = new IntakeIOFake();
    shooter =
        new Shooter(shooterIo) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
    hopper =
        new Hopper(hopperIo) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
    intake =
        new Intake(intakeIo) {
          @Override
          public Scheduler getRegisteredScheduler() {
            return scheduler;
          }
        };
    Pose2d robot = new Pose2d(2.0, 4.045, Rotation2d.ZERO);
    superstructure =
        new Superstructure(
            () -> robot,
            ChassisVelocities::new,
            robot::getRotation,
            shooter,
            hopper,
            intake,
            scheduler,
            () -> false);
  }

  @AfterEach
  void tearDown() {
    scheduler.cancelAll();
    shooter.close();
    hopper.close();
    intake.close();
  }

  @Test
  void idleAndIntakingAreImmediate() {
    assertEquals(Current.IDLE, Superstructure.nextState(Wanted.IDLE, Current.SHOOTING, ready()));
    assertEquals(
        Current.INTAKING, Superstructure.nextState(Wanted.INTAKING, Current.IDLE, notReady()));
  }

  @Test
  void shootingStartsAtSpinUpThenAdvancesWhenReady() {
    assertEquals(
        Current.SPINNING_UP, Superstructure.nextState(Wanted.SHOOTING, Current.IDLE, notReady()));
    assertEquals(
        Current.SPINNING_UP,
        Superstructure.nextState(Wanted.SHOOTING, Current.SPINNING_UP, notReady()));
    assertEquals(
        Current.SHOOTING, Superstructure.nextState(Wanted.SHOOTING, Current.SPINNING_UP, ready()));
  }

  @Test
  void headingDropReturnsToSpinUp() {
    assertEquals(
        Current.SPINNING_UP,
        Superstructure.nextState(Wanted.SHOOTING, Current.SHOOTING, new Inputs(true, true, true)));
    assertEquals(
        Current.SHOOTING,
        Superstructure.nextState(Wanted.SHOOTING, Current.SHOOTING, new Inputs(true, true, false)));
  }

  @Test
  void passingFamilyMirrorsShooting() {
    assertEquals(
        Current.SPINNING_UP, Superstructure.nextState(Wanted.PASSING, Current.IDLE, notReady()));
    assertEquals(
        Current.PASSING,
        Superstructure.nextState(Wanted.PASS_INTAKE, Current.SPINNING_UP, ready()));
  }

  @Test
  void dispatchSpinUpReversesFeeder() {
    superstructure.setWanted(Wanted.SHOOTING);
    tick();

    assertEquals(Current.SPINNING_UP, superstructure.getCurrent());
    assertEquals(ShooterIOFake.Request.VELOCITY, shooterIo.flywheelRequest);
    assertEquals(ShooterIOFake.Request.VOLTAGE, shooterIo.feederRequest);
    assertTrue(shooterIo.feederVoltageCommand.in(org.wpilib.units.Units.Volts) < 0.0);
  }

  @Test
  void dispatchShootWhenReady() {
    superstructure.setWanted(Wanted.SHOOTING);
    tick();
    shooterIo.flywheelFeedback = superstructure.getShotSetpoint().flywheelSpeed();
    for (int i = 0; i < Superstructure.HUB_CONFIRM_LOOPS + 1; i++) {
      tick();
    }

    assertEquals(Current.SHOOTING, superstructure.getCurrent());
    assertEquals(ShooterIOFake.Request.VELOCITY, shooterIo.feederRequest);
    assertTrue(shooterIo.feederVelocityCommand.in(RotationsPerSecond) > 0.0);
  }

  private void tick() {
    shooter.periodic();
    hopper.periodic();
    intake.periodic();
    superstructure.periodic();
    scheduler.run();
  }

  private static Inputs ready() {
    return new Inputs(true, true, false);
  }

  private static Inputs notReady() {
    return new Inputs(false, false, false);
  }

  private static void assertTrue(boolean value) {
    org.junit.jupiter.api.Assertions.assertTrue(value);
  }
}
