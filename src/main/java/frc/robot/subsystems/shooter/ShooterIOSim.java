package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.sim.VelocityPlant;
import org.wpilib.math.system.DCMotor;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Voltage;

/** WPILib flywheel plants for the shooter. Used in SIM only. */
public class ShooterIOSim implements ShooterIO {
  private static final double FLYWHEEL_MASS_KG = 1.134;
  private static final double FLYWHEEL_RADIUS_M = Units.inchesToMeters(2.0);
  private static final double FEEDER_MASS_KG = 0.227;
  private static final double FEEDER_RADIUS_M = Units.inchesToMeters(2.0);

  private final VelocityPlant flywheel =
      new VelocityPlant(
          DCMotor.getKrakenX60(1),
          0.5 * FLYWHEEL_MASS_KG * FLYWHEEL_RADIUS_M * FLYWHEEL_RADIUS_M,
          ShooterConstants.FLYWHEEL_GEARING);
  private final VelocityPlant feeder =
      new VelocityPlant(
          DCMotor.getKrakenX60(1),
          0.5 * FEEDER_MASS_KG * FEEDER_RADIUS_M * FEEDER_RADIUS_M,
          ShooterConstants.FEEDER_GEARING);

  @Override
  public void updateInputs(ShooterInputs inputs) {
    flywheel.update();
    feeder.update();

    inputs.flywheelTalonConnected = true;
    inputs.flywheelFollowerConnected = true;
    inputs.flywheelUsingBackup = false;
    inputs.flywheelVelocity = RotationsPerSecond.of(flywheel.velocityRadPerSec() / (2.0 * Math.PI));
    inputs.flywheelStatorCurrent = Amps.of(flywheel.currentAmps());
    inputs.flywheelAppliedVoltage = Volts.of(flywheel.appliedVolts());

    inputs.feederTalonConnected = true;
    inputs.feederVelocity = RotationsPerSecond.of(feeder.velocityRadPerSec() / (2.0 * Math.PI));
    inputs.feederStatorCurrent = Amps.of(feeder.currentAmps());
    inputs.feederAppliedVoltage = Volts.of(feeder.appliedVolts());
  }

  @Override
  public void setFlywheelVelocity(AngularVelocity targetVelocity) {
    flywheel.setVelocityRadPerSec(targetVelocity.in(RotationsPerSecond) * 2.0 * Math.PI);
  }

  @Override
  public void setFlywheelVoltage(Voltage voltage) {
    flywheel.setVoltage(voltage.in(Volts));
  }

  @Override
  public void setFeederVelocity(AngularVelocity targetVelocity) {
    feeder.setVelocityRadPerSec(targetVelocity.in(RotationsPerSecond) * 2.0 * Math.PI);
  }

  @Override
  public void setFeederVoltage(Voltage voltage) {
    feeder.setVoltage(voltage.in(Volts));
  }
}
