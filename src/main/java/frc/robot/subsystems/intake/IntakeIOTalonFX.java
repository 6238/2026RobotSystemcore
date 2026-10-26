package frc.robot.subsystems.intake;

import static frc.robot.util.PhoenixUtil.tryUntilOk;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.LeaderFailover;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class IntakeIOTalonFX implements IntakeIO {
  private enum RollerControl {
    NONE,
    VELOCITY,
    VOLTAGE
  }

  private final TalonFX pivotTalon;
  private final CANcoder cancoder;
  private final TalonFX rollerTalon;
  private final TalonFX rollerFollowerTalon;
  private final Follower followLead =
      new Follower(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_ALIGNMENT);
  private final Follower followBackup =
      new Follower(IntakeConstants.ROLLER_FOLLOWER_ID, IntakeConstants.ROLLER_ALIGNMENT);
  private final boolean opposed = IntakeConstants.ROLLER_ALIGNMENT == MotorAlignmentValue.Opposed;

  private final MotionMagicVoltage pivotPosition = new MotionMagicVoltage(0);
  private final VoltageOut pivotVoltage = new VoltageOut(0);
  private final VelocityVoltage rollerVelocity = new VelocityVoltage(0);
  private final VoltageOut rollerVoltage = new VoltageOut(0);

  private final Debouncer leadConnectedDebounce =
      new Debouncer(
          IntakeConstants.ROLLER_CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.FALLING);
  private final Debouncer backupConnectedDebounce =
      new Debouncer(
          IntakeConstants.ROLLER_CONNECTION_DEBOUNCE_SECONDS, Debouncer.DebounceType.FALLING);

  private final StatusSignalCollection signals;
  private final StatusSignal<Angle> pivotAngle;
  private final StatusSignal<AngularVelocity> pivotAngularVelocity;
  private final StatusSignal<Current> pivotSupplyCurrent;
  private final StatusSignal<Current> pivotStatorCurrent;
  private final StatusSignal<Voltage> pivotMotorVoltage;
  private final StatusSignal<AngularVelocity> leadAngularVelocity;
  private final StatusSignal<Current> leadSupplyCurrent;
  private final StatusSignal<Voltage> leadMotorVoltage;
  private final StatusSignal<AngularVelocity> backupAngularVelocity;
  private final StatusSignal<Current> backupSupplyCurrent;
  private final StatusSignal<Voltage> backupMotorVoltage;

  private boolean usingBackup;
  private RollerControl rollerControl = RollerControl.NONE;
  private AngularVelocity lastRollerVelocity = RotationsPerSecond.of(0.0);
  private Voltage lastRollerVoltage = Volts.of(0.0);

  public IntakeIOTalonFX() {
    pivotTalon = new TalonFX(IntakeConstants.PIVOT_MOTOR_ID, IntakeConstants.PIVOT_CAN_BUS);
    cancoder = new CANcoder(IntakeConstants.PIVOT_CANCODER_ID, IntakeConstants.PIVOT_CAN_BUS);
    rollerTalon = new TalonFX(IntakeConstants.ROLLER_MOTOR_ID, IntakeConstants.ROLLER_CAN_BUS);
    rollerFollowerTalon =
        new TalonFX(IntakeConstants.ROLLER_FOLLOWER_ID, IntakeConstants.ROLLER_CAN_BUS);

    tryUntilOk(5, () -> cancoder.getConfigurator().apply(IntakeConstants.cancoderConfig(), 0.25));
    tryUntilOk(5, () -> pivotTalon.getConfigurator().apply(IntakeConstants.pivotConfig(), 0.25));
    tryUntilOk(5, () -> rollerTalon.getConfigurator().apply(IntakeConstants.rollerConfig(), 0.25));
    tryUntilOk(
        5, () -> rollerFollowerTalon.getConfigurator().apply(IntakeConstants.rollerConfig(), 0.25));

    rollerFollowerTalon.setControl(followLead);

    pivotAngle = pivotTalon.getPosition();
    pivotAngularVelocity = pivotTalon.getVelocity();
    pivotSupplyCurrent = pivotTalon.getSupplyCurrent();
    pivotStatorCurrent = pivotTalon.getStatorCurrent();
    pivotMotorVoltage = pivotTalon.getMotorVoltage();
    leadAngularVelocity = rollerTalon.getVelocity();
    leadSupplyCurrent = rollerTalon.getSupplyCurrent();
    leadMotorVoltage = rollerTalon.getMotorVoltage();
    backupAngularVelocity = rollerFollowerTalon.getVelocity();
    backupSupplyCurrent = rollerFollowerTalon.getSupplyCurrent();
    backupMotorVoltage = rollerFollowerTalon.getMotorVoltage();

    signals =
        new StatusSignalCollection(
            pivotAngle,
            pivotAngularVelocity,
            pivotSupplyCurrent,
            pivotStatorCurrent,
            pivotMotorVoltage,
            leadAngularVelocity,
            leadSupplyCurrent,
            leadMotorVoltage,
            backupAngularVelocity,
            backupSupplyCurrent,
            backupMotorVoltage);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    signals.refreshAll();

    boolean leadConnected = leadConnectedDebounce.calculate(rollerTalon.isConnected());
    boolean backupConnected = backupConnectedDebounce.calculate(rollerFollowerTalon.isConnected());
    boolean wantBackup =
        LeaderFailover.nextUsingBackup(usingBackup, leadConnected, backupConnected);
    if (wantBackup != usingBackup) {
      usingBackup = wantBackup;
      applyFollowerRole();
      reapplyRoller();
    }

    inputs.pivotTalonConnected = pivotTalon.isConnected();
    inputs.pivotPosition = pivotAngle.getValue();
    inputs.pivotVelocity = pivotAngularVelocity.getValue();
    inputs.pivotSupplyCurrent = pivotSupplyCurrent.getValue();
    inputs.pivotStatorCurrent = pivotStatorCurrent.getValue();
    inputs.pivotAppliedVoltage = pivotMotorVoltage.getValue();

    inputs.rollerTalonConnected = leadConnected;
    inputs.rollerFollowerConnected = backupConnected;
    inputs.rollerUsingBackup = usingBackup;
    inputs.rollerVelocity =
        signed(usingBackup ? backupAngularVelocity.getValue() : leadAngularVelocity.getValue());
    inputs.rollerSupplyCurrent =
        usingBackup ? backupSupplyCurrent.getValue() : leadSupplyCurrent.getValue();
    inputs.rollerAppliedVoltage =
        signed(usingBackup ? backupMotorVoltage.getValue() : leadMotorVoltage.getValue());
  }

  @Override
  public void setPivotPosition(Angle targetAngle) {
    pivotTalon.setControl(pivotPosition.withPosition(targetAngle));
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    pivotTalon.setControl(pivotVoltage.withOutput(voltage));
  }

  @Override
  public void resetPivotAngle() {
    pivotTalon.setPosition(IntakeConstants.DOWN_ANGLE);
  }

  @Override
  public void setPivotBrakeMode(boolean brake) {
    var output = new MotorOutputConfigs();
    pivotTalon.getConfigurator().refresh(output);
    output.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    pivotTalon.getConfigurator().apply(output);
  }

  @Override
  public void setRollerVelocity(AngularVelocity targetVelocity) {
    rollerControl = RollerControl.VELOCITY;
    lastRollerVelocity = targetVelocity;
    commandTalon().setControl(rollerVelocity.withVelocity(signed(targetVelocity)));
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerControl = RollerControl.VOLTAGE;
    lastRollerVoltage = voltage;
    commandTalon().setControl(rollerVoltage.withOutput(signed(voltage)));
  }

  private void applyFollowerRole() {
    if (usingBackup) {
      rollerTalon.setControl(followBackup);
    } else {
      rollerFollowerTalon.setControl(followLead);
    }
  }

  private void reapplyRoller() {
    switch (rollerControl) {
      case VELOCITY -> setRollerVelocity(lastRollerVelocity);
      case VOLTAGE -> setRollerVoltage(lastRollerVoltage);
      case NONE -> {}
    }
  }

  private TalonFX commandTalon() {
    return usingBackup ? rollerFollowerTalon : rollerTalon;
  }

  private AngularVelocity signed(AngularVelocity velocity) {
    return usingBackup && opposed ? velocity.unaryMinus() : velocity;
  }

  private Voltage signed(Voltage voltage) {
    return usingBackup && opposed ? voltage.unaryMinus() : voltage;
  }
}
