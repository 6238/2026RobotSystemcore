package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * Records the last command sent across the IO boundary and lets tests inject feedback. No dynamics.
 */
public class ShooterIOFake implements ShooterIO {
  public enum Request {
    NONE,
    VELOCITY,
    VOLTAGE
  }

  public Request flywheelRequest = Request.NONE;
  public Request feederRequest = Request.NONE;
  public AngularVelocity flywheelVelocityCommand = RotationsPerSecond.of(0.0);
  public AngularVelocity feederVelocityCommand = RotationsPerSecond.of(0.0);
  public Voltage flywheelVoltageCommand = Volts.of(0.0);
  public Voltage feederVoltageCommand = Volts.of(0.0);

  public AngularVelocity flywheelFeedback = RotationsPerSecond.of(0.0);
  public AngularVelocity feederFeedback = RotationsPerSecond.of(0.0);
  public boolean beamBreakTriggered;

  @Override
  public void updateInputs(ShooterInputs inputs) {
    inputs.flywheelTalonConnected = true;
    inputs.flywheelFollowerConnected = true;
    inputs.flywheelUsingBackup = false;
    inputs.feederTalonConnected = true;
    inputs.flywheelVelocity = flywheelFeedback;
    inputs.feederVelocity = feederFeedback;
    inputs.flywheelAppliedVoltage = flywheelVoltageCommand;
    inputs.feederAppliedVoltage = feederVoltageCommand;
    inputs.beamBreakTriggered = beamBreakTriggered;
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity targetVelocity) {
    flywheelRequest = Request.VELOCITY;
    flywheelVelocityCommand = targetVelocity;
    flywheelVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheelRequest = Request.VOLTAGE;
    flywheelVoltageCommand = voltage;
    flywheelVelocityCommand = RotationsPerSecond.of(0.0);
  }

  @Override
  public void setFeederVelocity(AngularVelocity targetVelocity) {
    feederRequest = Request.VELOCITY;
    feederVelocityCommand = targetVelocity;
    feederVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setFeederVoltage(Voltage voltage) {
    feederRequest = Request.VOLTAGE;
    feederVoltageCommand = voltage;
    feederVelocityCommand = RotationsPerSecond.of(0.0);
  }
}
