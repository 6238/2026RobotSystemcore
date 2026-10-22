package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class ShooterIOTalonFX implements ShooterIO {
  private TalonFX leftTalon;
  private TalonFX rightTalon;
  private TalonFX feederTalon;

  private VelocityVoltage velocityVoltage = new VelocityVoltage(0);

  private StatusSignalCollection signals;
  private StatusSignal<AngularVelocity> flywheelAngularVelocity;
  private StatusSignal<Current> flywheelStatorCurrent;
  private StatusSignal<Voltage> flywheelVoltage;
  private StatusSignal<AngularVelocity> feederAngularVelocity;
  private StatusSignal<Current> feederStatorCurrent;
  private StatusSignal<Voltage> feederVoltage;

  public ShooterIOTalonFX() {
    leftTalon = new TalonFX(ShooterConstants.LEFT_TALON_ID, ShooterConstants.CAN_BUS);
    rightTalon = new TalonFX(ShooterConstants.RIGHT_TALON_ID, ShooterConstants.CAN_BUS);
    feederTalon = new TalonFX(ShooterConstants.FEEDER_TALON_ID, ShooterConstants.CAN_BUS);

    leftTalon.getConfigurator().apply(ShooterConstants.FLYWHEEL_SLOT_0);
    rightTalon.setControl(
        new Follower(ShooterConstants.LEFT_TALON_ID, ShooterConstants.MOTOR_ALLIGNMENT));
    feederTalon.getConfigurator().apply(ShooterConstants.FEEDER_SLOT_0);

    flywheelAngularVelocity = leftTalon.getVelocity();
    flywheelStatorCurrent = leftTalon.getStatorCurrent();
    flywheelVoltage = leftTalon.getMotorVoltage();
    
    feederAngularVelocity = feederTalon.getVelocity();
    feederStatorCurrent = feederTalon.getStatorCurrent();
    feederVoltage = feederTalon.getMotorVoltage();

    signals = new StatusSignalCollection(flywheelAngularVelocity, flywheelStatorCurrent, flywheelVoltage, feederAngularVelocity, feederStatorCurrent, feederVoltage);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    signals.refreshAll();

    inputs.leftTalonConnected = leftTalon.isConnected();
    inputs.rightTalonConnected = rightTalon.isConnected();

    inputs.shooterVelocity = flywheelAngularVelocity.getValue();
    inputs.shooterStatorCurrent = flywheelStatorCurrent.getValue();
    inputs.shooterAppliedVoltage = flywheelVoltage.getValue();

    inputs.feederVelocity = feederAngularVelocity.getValue();
    inputs.feederStatorCurrent = feederStatorCurrent.getValue();
    inputs.feederAppliedVoltage = feederVoltage.getValue();
  }

  public void setFlywheelVelocity(AngularVelocity targetVelocity) {
    leftTalon.setControl(velocityVoltage.withVelocity(targetVelocity));
  }

  public void idleFlywheel() {
    leftTalon.setVoltage(0.0);
  }

  public void setFeederVelocity(AngularVelocity targetVelocity) {
    feederTalon.setControl(velocityVoltage.withVelocity(targetVelocity));
  }

  public void idleFeeder() {
    feederTalon.setVoltage(0.0);
  }
}
