package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/**
 * Records the last command sent across the IO boundary and lets tests inject feedback. No dynamics.
 */
public class HopperIOFake implements HopperIO {
  public enum Request {
    NONE,
    VELOCITY,
    VOLTAGE
  }

  public Request indexerRequest = Request.NONE;
  public Request topIndexerRequest = Request.NONE;
  public AngularVelocity indexerVelocityCommand = RotationsPerSecond.of(0.0);
  public AngularVelocity topIndexerVelocityCommand = RotationsPerSecond.of(0.0);
  public Voltage indexerVoltageCommand = Volts.of(0.0);
  public Voltage topIndexerVoltageCommand = Volts.of(0.0);

  public AngularVelocity indexerFeedback = RotationsPerSecond.of(0.0);
  public AngularVelocity topIndexerFeedback = RotationsPerSecond.of(0.0);

  @Override
  public void updateInputs(HopperInputs inputs) {
    inputs.indexerTalonConnected = true;
    inputs.topIndexerTalonConnected = true;
    inputs.indexerVelocity = indexerFeedback;
    inputs.topIndexerVelocity = topIndexerFeedback;
    inputs.indexerAppliedVoltage = indexerVoltageCommand;
    inputs.topIndexerAppliedVoltage = topIndexerVoltageCommand;
  }

  @Override
  public void setIndexerVelocity(AngularVelocity targetVelocity) {
    indexerRequest = Request.VELOCITY;
    indexerVelocityCommand = targetVelocity;
    indexerVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setIndexerVoltage(Voltage voltage) {
    indexerRequest = Request.VOLTAGE;
    indexerVoltageCommand = voltage;
    indexerVelocityCommand = RotationsPerSecond.of(0.0);
  }

  @Override
  public void setTopIndexerVelocity(AngularVelocity targetVelocity) {
    topIndexerRequest = Request.VELOCITY;
    topIndexerVelocityCommand = targetVelocity;
    topIndexerVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setTopIndexerVoltage(Voltage voltage) {
    topIndexerRequest = Request.VOLTAGE;
    topIndexerVoltageCommand = voltage;
    topIndexerVelocityCommand = RotationsPerSecond.of(0.0);
  }
}
