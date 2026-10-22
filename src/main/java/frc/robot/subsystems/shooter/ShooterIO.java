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
    public boolean leftTalonConnected;
    public boolean rightTalonConnected;
    public AngularVelocity shooterVelocity = RotationsPerSecond.of(0.0);
    public Current shooterStatorCurrent = Amps.of(0.0);
    public Voltage shooterAppliedVoltage = Volts.of(0.0);

    public boolean feederTalonConnected;
    public AngularVelocity feederVelocity = RotationsPerSecond.of(0.0);
    public Current feederStatorCurrent = Amps.of(0.0);
    public Voltage feederAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(ShooterInputs inputs) {}

  public default void setFlywheelVelocity(AngularVelocity targetVelocity) {}
  public default void setFeederVelocity(AngularVelocity targetVelocity) {}

  public default void idleFlywheel() {}
  public default void idleFeeder() {}
}
