package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.Constants;
import frc.robot.sim.VelocityPlant;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.system.Models;
import org.wpilib.simulation.SingleJointedArmSim;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/** WPILib arm + flywheel plants for the combined intake. Used in SIM only. */
public class IntakeIOSim implements IntakeIO {
  private static final double ARM_MOI_KG_M2 = 0.04;
  private static final double ARM_LENGTH_M = 0.4;
  private static final double ROLLER_MASS_KG = 0.3;
  private static final double ROLLER_RADIUS_M = 0.025;

  private final DCMotor pivotMotor = DCMotor.getKrakenX60(1);
  private final SingleJointedArmSim pivot =
      new SingleJointedArmSim(
          Models.singleJointedArmFromPhysicalConstants(
              pivotMotor, ARM_MOI_KG_M2, IntakeConstants.PIVOT_GEARING),
          pivotMotor,
          IntakeConstants.PIVOT_GEARING,
          ARM_LENGTH_M,
          IntakeConstants.DOWN_ANGLE.in(Radians),
          IntakeConstants.START_ANGLE.in(Radians),
          true,
          IntakeConstants.START_ANGLE.in(Radians));
  private final VelocityPlant roller =
      new VelocityPlant(
          DCMotor.getKrakenX60(2),
          0.5 * ROLLER_MASS_KG * ROLLER_RADIUS_M * ROLLER_RADIUS_M,
          IntakeConstants.ROLLER_GEARING);

  private double pivotVolts;
  private Double pivotTargetRad;

  @Override
  public void updateInputs(IntakeInputs inputs) {
    if (pivotTargetRad != null) {
      double error = pivotTargetRad - pivot.getAngle();
      pivotVolts = Math.clamp(8.0 * error - 0.3 * pivot.getVelocity(), -12.0, 12.0);
    }
    pivot.setInputVoltage(pivotVolts);
    pivot.update(Constants.ROBOT_PERIOD);
    roller.update();

    inputs.pivotTalonConnected = true;
    inputs.pivotPosition = Radians.of(pivot.getAngle());
    inputs.pivotVelocity = RotationsPerSecond.of(pivot.getVelocity() / (2.0 * Math.PI));
    inputs.pivotSupplyCurrent = Amps.of(Math.abs(pivot.getCurrentDraw()));
    inputs.pivotStatorCurrent = inputs.pivotSupplyCurrent;
    inputs.pivotAppliedVoltage = Volts.of(pivotVolts);

    inputs.rollerTalonConnected = true;
    inputs.rollerFollowerConnected = true;
    inputs.rollerUsingBackup = false;
    inputs.rollerVelocity = RotationsPerSecond.of(roller.velocityRadPerSec() / (2.0 * Math.PI));
    inputs.rollerSupplyCurrent = Amps.of(roller.currentAmps());
    inputs.rollerAppliedVoltage = Volts.of(roller.appliedVolts());
  }

  @Override
  public void setPivotPosition(Angle targetAngle) {
    pivotTargetRad = targetAngle.in(Radians);
  }

  @Override
  public void setPivotVoltage(Voltage voltage) {
    pivotTargetRad = null;
    pivotVolts = Math.clamp(voltage.in(Volts), -12.0, 12.0);
  }

  @Override
  public void resetPivotAngle() {
    pivotTargetRad = IntakeConstants.DOWN_ANGLE.in(Radians);
  }

  @Override
  public void setRollerVelocity(AngularVelocity targetVelocity) {
    roller.setVelocityRadPerSec(targetVelocity.in(RotationsPerSecond) * 2.0 * Math.PI);
  }

  @Override
  public void setRollerVoltage(Voltage voltage) {
    roller.setVoltage(voltage.in(Volts));
  }
}
