package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Seconds;
import static org.wpilib.units.Units.Volts;

import frc.robot.util.Measures;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Pivot plus rollers. One mechanism because Superstructure states always command both, and Commands
 * v3 will not let two commands own the same lock.
 */
public class Intake implements Mechanism, AutoCloseable {
  private final IntakeIO io;
  private final IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();

  private Angle pivotSetpoint;
  private AngularVelocity rollerSetpoint;
  private boolean jamReversing;
  private double jamReverseStartSec = Double.NaN;
  private boolean brakeMode = true;

  private final Debouncer stallDebouncer =
      new Debouncer(IntakeConstants.STALL_DEBOUNCE_SECONDS, Debouncer.DebounceType.RISING);

  private final Alert pivotDisconnectedAlert =
      new Alert("Intake", "pivotDisconnected", "Intake pivot motor disconnected.", Level.HIGH);
  private final Alert rollerDisconnectedAlert =
      new Alert("Intake", "rollerDisconnected", "Intake roller motor disconnected.", Level.HIGH);
  private final Alert rollerFollowerDisconnectedAlert =
      new Alert(
          "Intake",
          "rollerFollowerDisconnected",
          "Intake roller follower disconnected.",
          Level.HIGH);
  private final Alert rollerBackupAlert =
      new Alert(
          "Intake",
          "rollerUsingBackup",
          "Intake roller running on follower; lead motor is out.",
          Level.HIGH);

  public Intake(IntakeIO io) {
    this.io = io;
    setDefaultCommand(idle());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);

    pivotDisconnectedAlert.set(!inputs.pivotTalonConnected);
    rollerDisconnectedAlert.set(!inputs.rollerTalonConnected);
    rollerFollowerDisconnectedAlert.set(!inputs.rollerFollowerConnected);
    rollerBackupAlert.set(inputs.rollerUsingBackup);

    Logger.recordOutput("Intake/PivotSetpoint", setpointOrZero(pivotSetpoint));
    Logger.recordOutput("Intake/RollerSetpoint", setpointOrZero(rollerSetpoint));
    Logger.recordOutput("Intake/PivotClosedLoop", pivotSetpoint != null);
    Logger.recordOutput("Intake/RollerClosedLoop", rollerSetpoint != null);
    Logger.recordOutput("Intake/JamReversing", jamReversing);
    Logger.recordOutput("Intake/RollerUsingBackup", inputs.rollerUsingBackup);
  }

  @Override
  public Command idle() {
    return runRepeatedly(
            () -> {
              pivotSetpoint = null;
              rollerSetpoint = null;
              jamReversing = false;
              io.setPivotVoltage(Volts.of(0.0));
              io.setRollerVoltage(Volts.of(0.0));
            })
        .named("Intake[IDLE]");
  }

  public Command intake() {
    return runRepeatedly(
            () -> {
              commandPivot(IntakeConstants.DOWN_ANGLE);
              commandRoller(IntakeConstants.ROLLER_SPEED);
            })
        .named("IntakeIntake");
  }

  public Command stow() {
    return runRepeatedly(
            () -> {
              commandPivot(IntakeConstants.UP_ANGLE);
              commandRoller(null);
            })
        .named("IntakeStow");
  }

  public Command ready() {
    return runRepeatedly(
            () -> {
              commandPivot(IntakeConstants.READY_ANGLE);
              commandRoller(null);
            })
        .named("IntakeReady");
  }

  public Command feed() {
    return run(coroutine -> {
          double startSec = Timer.getTimestamp();
          while (true) {
            commandPivot(Degrees.of(oscillateDegrees(Timer.getTimestamp() - startSec)));
            commandRoller(IntakeConstants.ROLLER_SPEED);
            coroutine.yield();
          }
        })
        .named("IntakeFeed");
  }

  public Command reverse() {
    return runRepeatedly(
            () -> {
              pivotSetpoint = null;
              jamReversing = false;
              io.setPivotVoltage(Volts.of(0.0));
              commandRoller(IntakeConstants.ROLLER_SPEED.unaryMinus());
            })
        .named("IntakeReverse");
  }

  public Command preload() {
    return run(coroutine -> {
          while (inputs.pivotSupplyCurrent.in(Amps)
              < IntakeConstants.PRELOAD_CURRENT_THRESHOLD_AMPS) {
            pivotSetpoint = null;
            rollerSetpoint = null;
            jamReversing = false;
            io.setPivotVoltage(Volts.of(IntakeConstants.PRELOAD_VOLTAGE_VOLTS));
            io.setRollerVoltage(Volts.of(0.0));
            coroutine.yield();
          }
        })
        .named("IntakePreload")
        .withTimeout(Seconds.of(IntakeConstants.PRELOAD_TIMEOUT_SECONDS));
  }

  public Command reset() {
    return run(coroutine -> io.resetPivotAngle()).named("IntakeReset");
  }

  public Command toggleBrakeMode() {
    return run(coroutine -> {
          brakeMode = !brakeMode;
          io.setPivotBrakeMode(brakeMode);
        })
        .named("IntakeToggleBrake");
  }

  public Angle getPivotPosition() {
    return inputs.pivotPosition;
  }

  /**
   * Sweeping asymmetric sine used while feeding. Extracted so the curve can be tested without
   * hardware.
   */
  public static double oscillateDegrees(double timeSec) {
    double downAngle =
        IntakeConstants.DOWN_ANGLE.in(Degrees) + IntakeConstants.OSCILLATE_DOWN_OFFSET_DEGREES;
    double upAngle = IntakeConstants.UP_ANGLE.in(Degrees);
    double centerMin = downAngle + IntakeConstants.OSCILLATE_CENTER_OFFSET_DEGREES;
    double centerMax = upAngle - IntakeConstants.OSCILLATE_CENTER_OFFSET_DEGREES;
    double center =
        Math.min(centerMin + IntakeConstants.OSCILLATE_SWEEP_RATE_DPS * timeSec, centerMax);

    double amplitude = IntakeConstants.OSCILLATE_AMPLITUDE_DEGREES;
    double frequency = IntakeConstants.OSCILLATE_FREQUENCY_HZ;
    double dutyCycle = Math.max(0.01, Math.min(0.99, IntakeConstants.OSCILLATE_DUTY_CYCLE));
    double period = 1.0 / frequency;
    double phaseInPeriod = (timeSec % period) / period;
    double sinArg;
    if (phaseInPeriod < dutyCycle) {
      sinArg = Math.PI * phaseInPeriod / dutyCycle;
    } else {
      sinArg = Math.PI + Math.PI * (phaseInPeriod - dutyCycle) / (1.0 - dutyCycle);
    }
    return Math.max(downAngle, Math.min(upAngle, center + amplitude * Math.sin(sinArg)));
  }

  @Override
  public void close() {
    pivotDisconnectedAlert.close();
    rollerDisconnectedAlert.close();
    rollerFollowerDisconnectedAlert.close();
    rollerBackupAlert.close();
  }

  private void commandPivot(Angle desired) {
    pivotSetpoint = clampPivot(desired);
    if (pivotSetpoint != null) {
      io.setPivotPosition(pivotSetpoint);
    } else {
      io.setPivotVoltage(Volts.of(0.0));
    }
  }

  private void commandRoller(AngularVelocity desired) {
    rollerSetpoint = finiteOrNull(desired);
    double now = Timer.getTimestamp();
    boolean aboveCutoff = inputs.pivotPosition.in(Degrees) > IntakeConstants.ROLLER_CUTOFF_DEGREES;

    boolean stallCondition =
        rollerSetpoint != null
            && rollerSetpoint.in(RotationsPerSecond) > 0.0
            && Math.abs(inputs.rollerVelocity.in(RotationsPerSecond))
                < IntakeConstants.STALL_VELOCITY_THRESHOLD_RPS
            && inputs.rollerSupplyCurrent.in(Amps) > IntakeConstants.STALL_CURRENT_THRESHOLD_AMPS;
    boolean stalled = stallDebouncer.calculate(stallCondition);

    if (stalled && !jamReversing) {
      jamReversing = true;
      jamReverseStartSec = now;
    }

    if (jamReversing) {
      boolean recovered =
          Math.abs(inputs.rollerVelocity.in(RotationsPerSecond))
              > IntakeConstants.STALL_VELOCITY_THRESHOLD_RPS;
      if (recovered
          && now - jamReverseStartSec >= IntakeConstants.JAM_REVERSE_MIN_DURATION_SECONDS) {
        jamReversing = false;
      } else {
        io.setRollerVoltage(Volts.of(-IntakeConstants.JAM_REVERSE_VOLTS));
        return;
      }
    }

    if (aboveCutoff) {
      io.setRollerVoltage(Volts.of(0.0));
      return;
    }

    if (rollerSetpoint != null) {
      io.setRollerVelocity(rollerSetpoint);
    } else {
      io.setRollerVoltage(Volts.of(0.0));
    }
  }

  private static Angle clampPivot(Angle angle) {
    if (!Measures.isFinite(angle)) {
      return null;
    }
    double degrees = angle.in(Degrees);
    double min = IntakeConstants.DOWN_ANGLE.in(Degrees);
    double max = IntakeConstants.START_ANGLE.in(Degrees);
    return Degrees.of(Math.min(max, Math.max(min, degrees)));
  }

  private static AngularVelocity finiteOrNull(AngularVelocity velocity) {
    return Measures.isFinite(velocity) ? velocity : null;
  }

  private static Angle setpointOrZero(Angle setpoint) {
    return setpoint != null ? setpoint : Degrees.of(0.0);
  }

  private static AngularVelocity setpointOrZero(AngularVelocity setpoint) {
    return setpoint != null ? setpoint : RotationsPerSecond.of(0.0);
  }
}
