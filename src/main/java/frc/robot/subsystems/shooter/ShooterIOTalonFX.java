package frc.robot.subsystems.shooter;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.robot.util.LeaderFailover;
import org.wpilib.hardware.discrete.DigitalInput;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private enum FlywheelControl {
    NONE,
    VELOCITY,
    VOLTAGE
  }

  private final TalonFX leftTalon;
  private final TalonFX rightTalon;
  private final TalonFX feederTalon;
  private final DigitalInput beamBreak;
  private final Follower follower =
      new Follower(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.MOTOR_ALIGNMENT);
  private final boolean opposed = ShooterConstants.MOTOR_ALIGNMENT == MotorAlignmentValue.Opposed;

  private final VelocityVoltage flywheelVelocity = new VelocityVoltage(0);
  private final VelocityVoltage feederVelocity = new VelocityVoltage(0);
  private final VoltageOut flywheelVoltage = new VoltageOut(0);
  private final VoltageOut feederVoltage = new VoltageOut(0);

  private final Debouncer leadConnectedDebounce =
      new Debouncer(
          ShooterConstants.FLYWHEEL_CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.FALLING);
  private final Debouncer backupConnectedDebounce =
      new Debouncer(
          ShooterConstants.FLYWHEEL_CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.FALLING);

  private final StatusSignalCollection signals;
  private final StatusSignal<AngularVelocity> leadAngularVelocity;
  private final StatusSignal<Current> leadStatorCurrent;
  private final StatusSignal<Voltage> leadMotorVoltage;
  private final StatusSignal<AngularVelocity> backupAngularVelocity;
  private final StatusSignal<Current> backupStatorCurrent;
  private final StatusSignal<Voltage> backupMotorVoltage;
  private final StatusSignal<AngularVelocity> feederAngularVelocity;
  private final StatusSignal<Current> feederStatorCurrent;
  private final StatusSignal<Voltage> feederMotorVoltage;

  private boolean usingBackup;
  private FlywheelControl flywheelControl = FlywheelControl.NONE;
  private AngularVelocity lastFlywheelVelocity = RotationsPerSecond.of(0.0);
  private Voltage lastFlywheelVoltage = Volts.of(0.0);

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(ShooterConstants.FLYWHEEL_MOTOR_ID, ShooterConstants.CAN_BUS);
    rightTalon = new TalonFX(ShooterConstants.FLYWHEEL_FOLLOWER_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_MOTOR_ID, ShooterConstants.CAN_BUS);
    beamBreak = new DigitalInput(ShooterConstants.BEAM_BREAK_CHANNEL);

    var flywheelConfig = ShooterConstants.flywheelConfig();
    var feederConfig = ShooterConstants.feederConfig();
    tryUntilOk(5, () -> leftTalon.getConfigurator().apply(flywheelConfig, 0.25));
    tryUntilOk(5, () -> rightTalon.getConfigurator().apply(flywheelConfig, 0.25));
    tryUntilOk(5, () -> feederTalon.getConfigurator().apply(feederConfig, 0.25));

    rightTalon.setControl(follower);

    leadAngularVelocity = leftTalon.getVelocity();
    leadStatorCurrent = leftTalon.getStatorCurrent();
    leadMotorVoltage = leftTalon.getMotorVoltage();
    backupAngularVelocity = rightTalon.getVelocity();
    backupStatorCurrent = rightTalon.getStatorCurrent();
    backupMotorVoltage = rightTalon.getMotorVoltage();
    feederAngularVelocity = feederTalon.getVelocity();
    feederStatorCurrent = feederTalon.getStatorCurrent();
    feederMotorVoltage = feederTalon.getMotorVoltage();

    signals =
        new StatusSignalCollection(
            leadAngularVelocity,
            leadStatorCurrent,
            leadMotorVoltage,
            backupAngularVelocity,
            backupStatorCurrent,
            backupMotorVoltage,
            feederAngularVelocity,
            feederStatorCurrent,
            feederMotorVoltage);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    signals.refreshAll();

    boolean leadConnected = leadConnectedDebounce.calculate(leftTalon.isConnected());
    boolean backupConnected = backupConnectedDebounce.calculate(rightTalon.isConnected());
    boolean wantBackup = LeaderFailover.useBackup(leadConnected, backupConnected);
    if (wantBackup != usingBackup) {
      usingBackup = wantBackup;
      applyFollowerRole();
      reapplyFlywheel();
    }

    inputs.flywheelTalonConnected = leadConnected;
    inputs.flywheelFollowerConnected = backupConnected;
    inputs.flywheelUsingBackup = usingBackup;
    inputs.feederTalonConnected = feederTalon.isConnected();

    AngularVelocity velocity =
        usingBackup ? backupAngularVelocity.getValue() : leadAngularVelocity.getValue();
    Voltage voltage = usingBackup ? backupMotorVoltage.getValue() : leadMotorVoltage.getValue();
    Current current = usingBackup ? backupStatorCurrent.getValue() : leadStatorCurrent.getValue();
    inputs.flywheelVelocity = signed(velocity);
    inputs.flywheelAppliedVoltage = signed(voltage);
    inputs.flywheelStatorCurrent = current;

    inputs.feederVelocity = feederAngularVelocity.getValue();
    inputs.feederStatorCurrent = feederStatorCurrent.getValue();
    inputs.feederAppliedVoltage = feederMotorVoltage.getValue();

    boolean broken = beamBreak.get();
    inputs.beamBreakTriggered = ShooterConstants.BEAM_BREAK_INVERTED ? broken : !broken;
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity targetVelocity) {
    flywheelControl = FlywheelControl.VELOCITY;
    lastFlywheelVelocity = targetVelocity;
    commandTalon().setControl(flywheelVelocity.withVelocity(signed(targetVelocity)));
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheelControl = FlywheelControl.VOLTAGE;
    lastFlywheelVoltage = voltage;
    commandTalon().setControl(flywheelVoltage.withOutput(signed(voltage)));
  }

  @Override
  public void setFeederVelocity(AngularVelocity targetVelocity) {
    feederTalon.setControl(feederVelocity.withVelocity(targetVelocity));
  }

  @Override
  public void setFeederVoltage(Voltage voltage) {
    feederTalon.setControl(feederVoltage.withOutput(voltage));
  }

  private void applyFollowerRole() {
    if (!usingBackup) {
      rightTalon.setControl(follower);
    }
  }

  private void reapplyFlywheel() {
    switch (flywheelControl) {
      case VELOCITY -> setFlywheelVelocity(lastFlywheelVelocity);
      case VOLTAGE -> setFlywheelVoltage(lastFlywheelVoltage);
      case NONE -> {}
    }
  }

  private TalonFX commandTalon() {
    return usingBackup ? rightTalon : leftTalon;
  }

  private AngularVelocity signed(AngularVelocity velocity) {
    return usingBackup && opposed ? velocity.unaryMinus() : velocity;
  }

  private Voltage signed(Voltage voltage) {
    return usingBackup && opposed ? voltage.unaryMinus() : voltage;
  }
}
