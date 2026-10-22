package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public interface ShooterIO {
  @AutoLog
  public class ShooterInputs {
    public boolean flywheelTalonConnected;
    public boolean flywheelFollowerConnected;
    public boolean flywheelUsingBackup;
    public AngularVelocity flywheelVelocity = RotationsPerSecond.of(0.0);
    public Current flywheelStatorCurrent = Amps.of(0.0);
    public Voltage flywheelAppliedVoltage = Volts.of(0.0);

    public boolean feederTalonConnected;
    public AngularVelocity feederVelocity = RotationsPerSecond.of(0.0);
    public Current feederStatorCurrent = Amps.of(0.0);
    public Voltage feederAppliedVoltage = Volts.of(0.0);

    public boolean beamBreakTriggered;
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setFlywheelVelocity(AngularVelocity targetVelocity) {}

  public default void setFlywheelVoltage(Voltage voltage) {}

  public default void setFeederVelocity(AngularVelocity targetVelocity) {}

  public default void setFeederVoltage(Voltage voltage) {}
}
