package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public interface HopperIO {
  @AutoLog
  public class HopperInputs {
    public boolean indexerTalonConnected;
    public AngularVelocity indexerVelocity = RotationsPerSecond.of(0.0);
    public Current indexerSupplyCurrent = Amps.of(0.0);
    public Voltage indexerAppliedVoltage = Volts.of(0.0);

    public boolean topIndexerTalonConnected;
    public AngularVelocity topIndexerVelocity = RotationsPerSecond.of(0.0);
    public Current topIndexerSupplyCurrent = Amps.of(0.0);
    public Voltage topIndexerAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(HopperInputs inputs) {}

  public default void setIndexerVelocity(AngularVelocity targetVelocity) {}

  public default void setIndexerVoltage(Voltage voltage) {}

  public default void setTopIndexerVelocity(AngularVelocity targetVelocity) {}

  public default void setTopIndexerVoltage(Voltage voltage) {}
}
