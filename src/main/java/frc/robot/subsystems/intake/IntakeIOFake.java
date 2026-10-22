package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

/**
 * Records the last command sent across the IO boundary and lets tests inject feedback. No dynamics.
 */
public class IntakeIOFake implements IntakeIO {
  public enum Request {
    NONE,
    POSITION,
    VELOCITY,
    VOLTAGE
  }

  public Request pivotRequest = Request.NONE;
  public Request rollerRequest = Request.NONE;
  public Angle pivotPositionCommand = Degrees.of(0.0);
  public Voltage pivotVoltageCommand = Volts.of(0.0);
  public AngularVelocity rollerVelocityCommand = RotationsPerSecond.of(0.0);
  public Voltage rollerVoltageCommand = Volts.of(0.0);

  public Angle pivotFeedback = Degrees.of(0.0);
  public AngularVelocity rollerFeedback = RotationsPerSecond.of(0.0);
  public Current rollerSupplyCurrent = Amps.of(0.0);
  public Current pivotSupplyCurrent = Amps.of(0.0);
  public boolean brakeMode = true;

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.pivotTalonConnected = true;
    inputs.rollerTalonConnected = true;
    inputs.rollerFollowerConnected = true;
    inputs.rollerUsingBackup = false;
    inputs.pivotPosition = pivotFeedback;
    inputs.pivotSupplyCurrent = pivotSupplyCurrent;
    inputs.pivotAppliedVoltage = pivotVoltageCommand;
    inputs.rollerVelocity = rollerFeedback;
    inputs.rollerSupplyCurrent = rollerSupplyCurrent;
    inputs.rollerAppliedVoltage = rollerVoltageCommand;
  }

  @Override
  public void setPivotPosition(Angle targetAngle) {
    pivotRequest = Request.POSITION;
    pivotPositionCommand = targetAngle;
    pivotVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    pivotRequest = Request.VOLTAGE;
    pivotVoltageCommand = voltage;
    pivotPositionCommand = Degrees.of(0.0);
  }

  @Override
  public void resetPivotAngle() {
    pivotFeedback = IntakeConstants.DOWN_ANGLE;
  }

  @Override
  public void setPivotBrakeMode(boolean brake) {
    brakeMode = brake;
  }

  @Override
  public void setRollerVelocity(AngularVelocity targetVelocity) {
    rollerRequest = Request.VELOCITY;
    rollerVelocityCommand = targetVelocity;
    rollerVoltageCommand = Volts.of(0.0);
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    rollerRequest = Request.VOLTAGE;
    rollerVoltageCommand = voltage;
    rollerVelocityCommand = RotationsPerSecond.of(0.0);
  }
}
