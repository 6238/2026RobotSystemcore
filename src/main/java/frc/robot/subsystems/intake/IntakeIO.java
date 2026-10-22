package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.littletonrobotics.junction.AutoLog;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public interface IntakeIO {
  @AutoLog
  public class IntakeInputs {
    public boolean pivotTalonConnected;
    public Angle pivotPosition = Degrees.of(0.0);
    public AngularVelocity pivotVelocity = RotationsPerSecond.of(0.0);
    public Current pivotSupplyCurrent = Amps.of(0.0);
    public Current pivotStatorCurrent = Amps.of(0.0);
    public Voltage pivotAppliedVoltage = Volts.of(0.0);

    public boolean rollerTalonConnected;
    public boolean rollerFollowerConnected;
    public boolean rollerUsingBackup;
    public AngularVelocity rollerVelocity = RotationsPerSecond.of(0.0);
    public Current rollerSupplyCurrent = Amps.of(0.0);
    public Voltage rollerAppliedVoltage = Volts.of(0.0);
  }

  public default void updateInputs(IntakeInputs inputs) {}

  public default void setPivotPosition(Angle targetAngle) {}

  public default void setPivotVoltage(Voltage voltage) {}

  public default void resetPivotAngle() {}

  public default void setPivotBrakeMode(boolean brake) {}

  public default void setRollerVelocity(AngularVelocity targetVelocity) {}

  public default void setRollerVoltage(Voltage voltage) {}
}
