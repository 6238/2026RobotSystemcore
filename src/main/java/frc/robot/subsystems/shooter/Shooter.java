package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.util.Measures;
import frc.robot.util.PulseWindow;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.system.Timer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/**
 * Flywheel plus feeder. One mechanism because Superstructure states always command both, and
 * Commands v3 will not let two commands own the same lock.
 */
public class Shooter implements Mechanism, AutoCloseable {
  private final ShooterIO io;
  private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
  private final PulseWindow beamBreak = new PulseWindow();

  /** Non-null only while that actuator is in velocity control. */
  private AngularVelocity flywheelSetpoint;

  private AngularVelocity feederSetpoint;

  private final Alert flywheelDisconnectedAlert =
      new Alert(
          "Shooter", "flywheelDisconnected", "Shooter flywheel motor disconnected.", Level.HIGH);
  private final Alert flywheelFollowerDisconnectedAlert =
      new Alert(
          "Shooter",
          "flywheelFollowerDisconnected",
          "Shooter flywheel follower disconnected.",
          Level.HIGH);
  private final Alert feederDisconnectedAlert =
      new Alert("Shooter", "feederDisconnected", "Shooter feeder motor disconnected.", Level.HIGH);
  private final Alert flywheelBackupAlert =
      new Alert(
          "Shooter",
          "flywheelUsingBackup",
          "Shooter flywheel running on follower; lead motor is out.",
          Level.HIGH);

  public Shooter(ShooterIO io) {
    this.io = io;
    setDefaultCommand(idle());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    beamBreak.update(inputs.beamBreakTriggered, Timer.getTimestamp());

    flywheelDisconnectedAlert.set(!inputs.flywheelTalonConnected);
    flywheelFollowerDisconnectedAlert.set(!inputs.flywheelFollowerConnected);
    feederDisconnectedAlert.set(!inputs.feederTalonConnected);
    flywheelBackupAlert.set(inputs.flywheelUsingBackup);

    Logger.recordOutput("Shooter/FlywheelSetpoint", setpointOrZero(flywheelSetpoint));
    Logger.recordOutput("Shooter/FeederSetpoint", setpointOrZero(feederSetpoint));
    Logger.recordOutput("Shooter/FlywheelClosedLoop", flywheelSetpoint != null);
    Logger.recordOutput("Shooter/FeederClosedLoop", feederSetpoint != null);
    Logger.recordOutput("Shooter/IsShooting", isShooting());
    Logger.recordOutput("Shooter/FlywheelUsingBackup", inputs.flywheelUsingBackup);
  }

  @Override
  public Command idle() {
    return runRepeatedly(
            () -> {
              flywheelSetpoint = null;
              feederSetpoint = null;
              io.setFlywheelVoltage(Volts.of(0.0));
              io.setFeederVoltage(Volts.of(0.0));
            })
        .named("Shooter[IDLE]");
  }

  public Command holdFlywheel(AngularVelocity velocity) {
    return holdFlywheel(() -> velocity);
  }

  public Command holdFlywheel(Supplier<AngularVelocity> velocity) {
    return runRepeatedly(
            () -> {
              flywheelSetpoint = finiteOrNull(velocity.get());
              feederSetpoint = null;
              if (flywheelSetpoint != null) {
                io.setFlywheelVelocity(flywheelSetpoint);
              } else {
                io.setFlywheelVoltage(Volts.of(0.0));
              }
              io.setFeederVoltage(Volts.of(0.0));
            })
        .named("ShooterHoldFlywheel");
  }

  public Command spinUp(AngularVelocity flywheelVelocity) {
    return spinUp(() -> flywheelVelocity);
  }

  public Command spinUp(Supplier<AngularVelocity> flywheelVelocity) {
    return runRepeatedly(
            () -> {
              flywheelSetpoint = finiteOrNull(flywheelVelocity.get());
              feederSetpoint = null;
              if (flywheelSetpoint != null) {
                io.setFlywheelVelocity(flywheelSetpoint);
              } else {
                io.setFlywheelVoltage(Volts.of(0.0));
              }
              io.setFeederVoltage(Volts.of(-ShooterConstants.FEEDER_REVERSE_VOLTS));
            })
        .named("ShooterSpinUp");
  }

  public Command shoot(AngularVelocity flywheelVelocity, AngularVelocity feederVelocity) {
    return shoot(() -> flywheelVelocity, () -> feederVelocity);
  }

  public Command shoot(
      Supplier<AngularVelocity> flywheelVelocity, Supplier<AngularVelocity> feederVelocity) {
    return runRepeatedly(
            () -> {
              flywheelSetpoint = finiteOrNull(flywheelVelocity.get());
              feederSetpoint = finiteOrNull(feederVelocity.get());
              if (flywheelSetpoint != null) {
                io.setFlywheelVelocity(flywheelSetpoint);
              } else {
                io.setFlywheelVoltage(Volts.of(0.0));
              }
              if (feederSetpoint != null) {
                io.setFeederVelocity(feederSetpoint);
              } else {
                io.setFeederVoltage(Volts.of(0.0));
              }
            })
        .named("ShooterShoot");
  }

  public Command reverse() {
    return runRepeatedly(
            () -> {
              flywheelSetpoint = null;
              feederSetpoint = null;
              io.setFlywheelVoltage(Volts.of(-ShooterConstants.FEEDER_REVERSE_VOLTS));
              io.setFeederVoltage(Volts.of(-ShooterConstants.FEEDER_REVERSE_VOLTS));
            })
        .named("ShooterReverse");
  }

  public boolean flywheelAtSpeed(AngularVelocity tolerance) {
    return flywheelSetpoint != null && inputs.flywheelVelocity.isNear(flywheelSetpoint, tolerance);
  }

  public boolean feederAtSpeed(AngularVelocity tolerance) {
    return feederSetpoint != null && inputs.feederVelocity.isNear(feederSetpoint, tolerance);
  }

  public AngularVelocity getFlywheelVelocity() {
    return inputs.flywheelVelocity;
  }

  public AngularVelocity getFeederVelocity() {
    return inputs.feederVelocity;
  }

  /** True if the beam break fired within {@link ShooterConstants#SHOOTING_WINDOW_SECONDS}. */
  public boolean isShooting() {
    return beamBreak.active(Timer.getTimestamp(), ShooterConstants.SHOOTING_WINDOW_SECONDS);
  }

  public double timeSinceBeamBreakSec() {
    return beamBreak.elapsedSec(Timer.getTimestamp());
  }

  @Override
  public void close() {
    flywheelDisconnectedAlert.close();
    flywheelFollowerDisconnectedAlert.close();
    feederDisconnectedAlert.close();
    flywheelBackupAlert.close();
  }

  private static AngularVelocity finiteOrNull(AngularVelocity velocity) {
    return Measures.isFinite(velocity) ? velocity : null;
  }

  private static AngularVelocity setpointOrZero(AngularVelocity setpoint) {
    return setpoint != null ? setpoint : RotationsPerSecond.of(0.0);
  }
}
