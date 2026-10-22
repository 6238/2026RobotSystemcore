package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.RadiansPerSecond;

import org.ironmaple.simulation.drivesims.GyroSimulation;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.system.Timer;

/** MapleSim gyro readings, including the library's modeled drift. */
public class GyroIOSim implements GyroIO {
  private final GyroSimulation gyro;

  public GyroIOSim(GyroSimulation gyro) {
    this.gyro = gyro;
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = gyro.getGyroReading();
    inputs.yawVelocityRadPerSec = gyro.getMeasuredAngularVelocity().in(RadiansPerSecond);
    inputs.odometryYawTimestamps = new double[] {Timer.getMonotonicTimestamp()};
    inputs.odometryYawPositions = new Rotation2d[] {inputs.yawPosition};
  }
}
