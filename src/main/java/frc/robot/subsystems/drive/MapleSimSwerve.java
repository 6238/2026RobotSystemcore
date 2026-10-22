package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Pounds;

import frc.robot.generated.RobotTunerConstants;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.system.DCMotor;

public class MapleSimSwerve {
  @SuppressWarnings("unchecked")
  public static SwerveDriveSimulation createSimulationDrive(RobotTunerConstants constants) {
    return createSimulationDrive(constants, new Pose2d(2.0, 2.0, Rotation2d.ZERO));
  }

  @SuppressWarnings("unchecked")
  public static SwerveDriveSimulation createSimulationDrive(
      RobotTunerConstants constants, Pose2d pose) {
    SwerveDriveSimulation sim =
        new SwerveDriveSimulation(
            new DriveTrainSimulationConfig(
                Pounds.of(90),
                Inches.of(27),
                Inches.of(27),
                Meters.of(constants.FrontLeft.LocationX - constants.BackLeft.LocationX),
                Meters.of(constants.FrontLeft.LocationY - constants.FrontRight.LocationY),
                COTS.ofPigeon2(),
                COTS.ofSwerveX2(DCMotor.getKrakenX60(1), DCMotor.getKrakenX60(1), 1.4, 2, 11)),
            pose);
    SimulatedArena.getInstance().addDriveTrainSimulation(sim);
    return sim;
  }
}
