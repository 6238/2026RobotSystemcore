package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.sim.VelocityPlant;
import org.wpilib.math.system.DCMotor;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/** WPILib flywheel plants for both indexers. Used in SIM only. */
public class HopperIOSim implements HopperIO {
  private static final double INDEXER_MASS_KG = 0.2;
  private static final double INDEXER_RADIUS_M = 0.025;
  private static final double INDEXER_MOI =
      0.5 * INDEXER_MASS_KG * INDEXER_RADIUS_M * INDEXER_RADIUS_M;

  private final VelocityPlant indexer =
      new VelocityPlant(DCMotor.getKrakenX60(1), INDEXER_MOI, HopperConstants.INDEXER_GEARING);
  private final VelocityPlant topIndexer =
      new VelocityPlant(DCMotor.getKrakenX60(1), INDEXER_MOI, HopperConstants.TOP_INDEXER_GEARING);

  @Override
  public void updateInputs(HopperInputs inputs) {
    indexer.update();
    topIndexer.update();

    inputs.indexerTalonConnected = true;
    inputs.indexerVelocity = RotationsPerSecond.of(indexer.velocityRadPerSec() / (2.0 * Math.PI));
    inputs.indexerSupplyCurrent = Amps.of(indexer.currentAmps());
    inputs.indexerAppliedVoltage = Volts.of(indexer.appliedVolts());

    inputs.topIndexerTalonConnected = true;
    inputs.topIndexerVelocity =
        RotationsPerSecond.of(topIndexer.velocityRadPerSec() / (2.0 * Math.PI));
    inputs.topIndexerSupplyCurrent = Amps.of(topIndexer.currentAmps());
    inputs.topIndexerAppliedVoltage = Volts.of(topIndexer.appliedVolts());
  }

  @Override
  public void setIndexerVelocity(AngularVelocity targetVelocity) {
    indexer.setVelocityRadPerSec(targetVelocity.in(RotationsPerSecond) * 2.0 * Math.PI);
  }

  @Override
  public void setIndexerVoltage(Voltage voltage) {
    indexer.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setTopIndexerVelocity(AngularVelocity targetVelocity) {
    topIndexer.setVelocityRadPerSec(targetVelocity.in(RotationsPerSecond) * 2.0 * Math.PI);
  }

  @Override
  public void setTopIndexerVoltage(Voltage voltage) {
    topIndexer.setVoltage(voltage.in(Volts));
  }
}
