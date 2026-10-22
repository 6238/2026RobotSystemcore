package frc.robot.subsystems.superstructure;

import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.hopper.Hopper;
import frc.robot.subsystems.hopper.HopperConstants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.subsystems.superstructure.ShotPlanner.ShotSetpoint;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.system.Timer;
import org.wpilib.units.Units;
import org.wpilib.units.measure.AngularVelocity;

/**
 * Coordinator only: reads mechanism queries, picks the next state, and schedules mechanism
 * commands. It never writes actuators.
 */
public class Superstructure {
  public enum Wanted {
    IDLE,
    INTAKING,
    SHOOTING,
    PASSING,
    SHOOT_INTAKE,
    PASS_INTAKE,
    PIT_SHOOT,
    REVERSE,
    SPIT
  }

  public enum Current {
    IDLE,
    INTAKING,
    SPINNING_UP,
    SHOOTING,
    PASSING,
    PIT_SHOOTING,
    REVERSE,
    SPIT
  }

  public record Inputs(boolean flywheelReady, boolean hubConfirmed, boolean headingDropped) {}

  record Mode(Current current, Wanted wanted, boolean holdFlywheel) {}

  public static final int HUB_CONFIRM_LOOPS = 2;
  public static final double POST_SHOT_HOLD_SECONDS = 0.5;

  private final Supplier<Pose2d> pose;
  private final Supplier<ChassisVelocities> speeds;
  private final Supplier<Rotation2d> rotation;
  private final Shooter shooter;
  private final Hopper hopper;
  private final Intake intake;
  private final Scheduler scheduler;
  private final BooleanSupplier hubSpinupActive;

  private Wanted wanted = Wanted.IDLE;
  private Current current = Current.IDLE;
  private Mode lastMode;
  private ShotSetpoint shotSetpoint = new ShotSetpoint();
  private int hubInToleranceLoops;
  private double postShotHoldStartSec = Double.NEGATIVE_INFINITY;
  private AngularVelocity hopperIndexerSpeed = HopperConstants.INDEXER_SPEED;
  private AngularVelocity hopperTopSpeed = HopperConstants.TOP_INDEXER_SPEED;

  public Superstructure(Drive drive, Shooter shooter, Hopper hopper, Intake intake) {
    this(
        drive::getPose,
        drive::getChassisSpeeds,
        drive::getRotation,
        shooter,
        hopper,
        intake,
        Scheduler.getDefault(),
        () -> false);
  }

  public Superstructure(
      Supplier<Pose2d> pose,
      Supplier<ChassisVelocities> speeds,
      Supplier<Rotation2d> rotation,
      Shooter shooter,
      Hopper hopper,
      Intake intake,
      Scheduler scheduler,
      BooleanSupplier hubSpinupActive) {
    this.pose = pose;
    this.speeds = speeds;
    this.rotation = rotation;
    this.shooter = shooter;
    this.hopper = hopper;
    this.intake = intake;
    this.scheduler = scheduler;
    this.hubSpinupActive = hubSpinupActive;
  }

  public void setWanted(Wanted wanted) {
    this.wanted = wanted;
  }

  public Command want(Wanted state) {
    return Command.noRequirements(coroutine -> wanted = state).named("Want[" + state + "]");
  }

  public Command want(Supplier<Wanted> state) {
    return Command.noRequirements(
            coroutine -> {
              while (true) {
                wanted = state.get();
                coroutine.yield();
              }
            })
        .whenCanceled(() -> wanted = Wanted.IDLE)
        .named("WantWhile");
  }

  public Wanted getWanted() {
    return wanted;
  }

  public Current getCurrent() {
    return current;
  }

  public ShotSetpoint getShotSetpoint() {
    return shotSetpoint;
  }

  public static Current nextState(Wanted wanted, Current current, Inputs inputs) {
    return switch (wanted) {
      case IDLE -> Current.IDLE;
      case INTAKING -> Current.INTAKING;
      case PIT_SHOOT -> Current.PIT_SHOOTING;
      case REVERSE -> Current.REVERSE;
      case SPIT -> Current.SPIT;
      case SHOOTING, SHOOT_INTAKE -> shootFamily(current, inputs, Current.SHOOTING);
      case PASSING, PASS_INTAKE -> shootFamily(current, inputs, Current.PASSING);
    };
  }

  public void periodic() {
    updateShotSetpoint();
    updateHopperSpeeds();
    updateHubToleranceLoops();

    Inputs inputs = readInputs();
    Current next = nextState(wanted, current, inputs);
    if (next != current) {
      if (isShootingFamily(current) && next == Current.IDLE) {
        postShotHoldStartSec = Timer.getTimestamp();
      }
      if (next == Current.SPINNING_UP) {
        hubInToleranceLoops = 0;
      }
      current = next;
    }

    Mode mode = new Mode(current, wanted, shouldHoldFlywheel());
    if (!mode.equals(lastMode)) {
      lastMode = mode;
      dispatch(mode);
    }

    Logger.recordOutput("Superstructure/Wanted", wanted);
    Logger.recordOutput("Superstructure/Current", current);
    Logger.recordOutput("Superstructure/HoldFlywheel", mode.holdFlywheel());
    Logger.recordOutput("Superstructure/HubInToleranceLoops", hubInToleranceLoops);
    Logger.recordOutput("Superstructure/ShotTarget", shotSetpoint.robotPose());
    Logger.recordOutput("Superstructure/FlywheelSetpoint", shotSetpoint.flywheelSpeed());
  }

  private static Current shootFamily(Current current, Inputs inputs, Current shooting) {
    if (current == shooting) {
      return inputs.headingDropped() ? Current.SPINNING_UP : shooting;
    }
    if (current == Current.SPINNING_UP) {
      return inputs.flywheelReady() && inputs.hubConfirmed() ? shooting : Current.SPINNING_UP;
    }
    return Current.SPINNING_UP;
  }

  private Inputs readInputs() {
    boolean passing = wanted == Wanted.PASSING || wanted == Wanted.PASS_INTAKE;
    boolean flywheelReady =
        passing
            ? shooter.getFlywheelVelocity().in(Units.RotationsPerSecond)
                >= ShooterConstants.PASSING_FLYWHEEL_MIN_RPS
            : shooter.flywheelAtSpeed(ShooterConstants.FLYWHEEL_SPINUP_TRANSITION_TOLERANCE);
    boolean headingDropped =
        hubRotationErrorDeg()
            > (passing
                    ? ShooterConstants.HUB_PASSING_DROPBACK_TOLERANCE
                    : ShooterConstants.HUB_DROPBACK_TOLERANCE)
                .in(Units.Degrees);
    return new Inputs(flywheelReady, hubInToleranceLoops >= HUB_CONFIRM_LOOPS, headingDropped);
  }

  private void updateShotSetpoint() {
    Pose2d robot = pose.get();
    ChassisVelocities robotSpeeds = speeds.get();
    switch (wanted) {
      case SHOOTING, SHOOT_INTAKE ->
          shotSetpoint = ShotPlanner.createShotSetpoint(Constants.hubPose(), robot, robotSpeeds);
      case PASSING, PASS_INTAKE ->
          shotSetpoint =
              ShotPlanner.createPassSetpoint(Constants.passPose(robot), robot, robotSpeeds);
      case PIT_SHOOT ->
          shotSetpoint =
              new ShotSetpoint(
                  ShooterConstants.SPINUP_FLYWHEEL_SPEED,
                  ShooterConstants.FEEDER_SPEED,
                  robot,
                  robot.getTranslation(),
                  robotSpeeds);
      default -> {}
    }
  }

  private void updateHopperSpeeds() {
    boolean tooClose =
        current == Current.SHOOTING
            && pose.get().getTranslation().getDistance(Constants.hubPose().getTranslation())
                < ShooterConstants.MIN_SHOT_DISTANCE_METERS;
    ChassisVelocities robotSpeeds = speeds.get();
    boolean onMove =
        Math.hypot(robotSpeeds.vx, robotSpeeds.vy) > ShooterConstants.HUB_HIGH_ROBOT_SPEED_MPS;
    double scale = tooClose ? 0.0 : (onMove ? HopperConstants.ON_MOVE_SCALE : 1.0);
    hopperIndexerSpeed = HopperConstants.INDEXER_SPEED.times(scale);
    hopperTopSpeed = HopperConstants.TOP_INDEXER_SPEED.times(scale);
  }

  private void updateHubToleranceLoops() {
    if (current != Current.SPINNING_UP) {
      return;
    }
    boolean passing = wanted == Wanted.PASSING || wanted == Wanted.PASS_INTAKE;
    boolean flywheelReady =
        passing
            ? shooter.getFlywheelVelocity().in(Units.RotationsPerSecond)
                >= ShooterConstants.PASSING_FLYWHEEL_MIN_RPS
            : shooter.flywheelAtSpeed(ShooterConstants.FLYWHEEL_SPINUP_TRANSITION_TOLERANCE);
    if (flywheelReady && hubInTolerance()) {
      hubInToleranceLoops++;
    } else {
      hubInToleranceLoops = 0;
    }
  }

  private boolean hubInTolerance() {
    return hubRotationErrorDeg() < hubToleranceDeg();
  }

  private double hubRotationErrorDeg() {
    return Math.abs(rotation.get().minus(shotSetpoint.robotPose().getRotation()).getDegrees());
  }

  private double hubToleranceDeg() {
    double distance = pose.get().getTranslation().getDistance(shotSetpoint.target());
    ChassisVelocities robotSpeeds = speeds.get();
    boolean tight =
        distance < ShooterConstants.HUB_NEAR_DISTANCE_METERS
            || Math.hypot(robotSpeeds.vx, robotSpeeds.vy)
                > ShooterConstants.HUB_HIGH_ROBOT_SPEED_MPS;
    return (tight
            ? ShooterConstants.HUB_ROTATION_TOLERANCE_TIGHT
            : ShooterConstants.HUB_ROTATION_TOLERANCE)
        .in(Units.Degrees);
  }

  private boolean shouldHoldFlywheel() {
    if (current != Current.IDLE && current != Current.INTAKING) {
      return false;
    }
    if (Timer.getTimestamp() - postShotHoldStartSec < POST_SHOT_HOLD_SECONDS) {
      return true;
    }
    return ShooterConstants.SPINUP_WHEN_HUB_ACTIVE && hubSpinupActive.getAsBoolean();
  }

  private void dispatch(Mode mode) {
    boolean intakeDown =
        mode.wanted() == Wanted.SHOOT_INTAKE || mode.wanted() == Wanted.PASS_INTAKE;
    switch (mode.current()) {
      case IDLE -> {
        scheduler.schedule(
            mode.holdFlywheel() ? shooter.holdFlywheel(this::holdSpeed) : shooter.idle());
        scheduler.schedule(hopper.idle());
        scheduler.schedule(intake.idle());
      }
      case INTAKING -> {
        scheduler.schedule(
            mode.holdFlywheel() ? shooter.holdFlywheel(this::holdSpeed) : shooter.idle());
        scheduler.schedule(hopper.idle());
        scheduler.schedule(intake.intake());
      }
      case SPINNING_UP -> {
        scheduler.schedule(shooter.spinUp(() -> shotSetpoint.flywheelSpeed()));
        scheduler.schedule(
            hopper.feed(
                HopperConstants.SPINUP_REVERSE_SPEED.unaryMinus(),
                HopperConstants.SPINUP_REVERSE_SPEED.unaryMinus()));
        scheduler.schedule(intakeDown ? intake.intake() : intake.ready());
      }
      case SHOOTING, PASSING, PIT_SHOOTING -> {
        scheduler.schedule(
            shooter.shoot(() -> shotSetpoint.flywheelSpeed(), () -> shotSetpoint.feederSpeed()));
        scheduler.schedule(hopper.feed(() -> hopperIndexerSpeed, () -> hopperTopSpeed));
        scheduler.schedule(intakeDown ? intake.intake() : intake.feed());
      }
      case REVERSE -> {
        scheduler.schedule(shooter.reverse());
        scheduler.schedule(hopper.reverse());
        scheduler.schedule(intake.idle());
      }
      case SPIT -> {
        scheduler.schedule(shooter.idle());
        scheduler.schedule(hopper.reverse());
        scheduler.schedule(intake.reverse());
      }
    }
  }

  private AngularVelocity holdSpeed() {
    if (isShootingFamily(current)
        || Timer.getTimestamp() - postShotHoldStartSec < POST_SHOT_HOLD_SECONDS) {
      return shotSetpoint.flywheelSpeed();
    }
    return ShooterConstants.SPINUP_FLYWHEEL_SPEED;
  }

  private static boolean isShootingFamily(Current current) {
    return current == Current.SHOOTING
        || current == Current.PASSING
        || current == Current.SPINNING_UP
        || current == Current.PIT_SHOOTING;
  }
}
