package frc.robot.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RadiansPerSecond;
import static org.wpilib.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController.GenericMotorController;
import org.wpilib.math.controller.PIDController;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.util.Units;
import org.wpilib.system.Timer;

/**
 * MapleSim-backed physics sim implementation of module IO.
 *
 * <p>Open-loop setters take volts. Closed-loop uses local PID plus drive/steer feedforward.
 * Voltages are applied here; {@code SimulatedArena} steps the plant after the scheduler.
 */
public class ModuleIOSim implements ModuleIO {
  private static final double DRIVE_KP = 0.2;
  private static final double DRIVE_KD = 0.0;
  private static final double DRIVE_KS = 0.0;
  private static final double DRIVE_KV_ROT = 0.91035;
  private static final double DRIVE_KV = 1.0 / Units.rotationsToRadians(1.0 / DRIVE_KV_ROT);
  private static final double TURN_KP = 8.0;
  private static final double TURN_KD = 0.0;
  private static final double TURN_KV = 2.33 / (2.0 * Math.PI);

  private final SwerveModuleSimulation moduleSimulation;
  private final GenericMotorController driveMotor;
  private final GenericMotorController turnMotor;

  private boolean driveClosedLoop = false;
  private boolean turnClosedLoop = false;
  private final PIDController driveController = new PIDController(DRIVE_KP, 0.0, DRIVE_KD);
  private final PIDController turnController = new PIDController(TURN_KP, 0.0, TURN_KD);
  private double driveFFVolts = 0.0;
  private double turnFFVolts = 0.0;
  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  public ModuleIOSim(SwerveModuleSimulation moduleSimulation) {
    this.moduleSimulation = moduleSimulation;
    this.driveMotor =
        moduleSimulation.useGenericMotorControllerForDrive().withCurrentLimit(Amps.of(60));
    this.turnMotor = moduleSimulation.useGenericControllerForSteer().withCurrentLimit(Amps.of(20));
    turnController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    double driveVelRadPerSec = moduleSimulation.getDriveWheelFinalSpeed().in(RadiansPerSecond);
    double turnPosRad = moduleSimulation.getSteerAbsoluteFacing().getRadians();

    if (driveClosedLoop) {
      driveAppliedVolts = driveFFVolts + driveController.calculate(driveVelRadPerSec);
    } else {
      driveController.reset();
    }
    if (turnClosedLoop) {
      turnAppliedVolts = turnFFVolts + turnController.calculate(turnPosRad);
    } else {
      turnController.reset();
    }

    double driveVoltsClamped = Math.clamp(driveAppliedVolts, -12.0, 12.0);
    double turnVoltsClamped = Math.clamp(turnAppliedVolts, -12.0, 12.0);
    driveMotor.requestVoltage(Volts.of(driveVoltsClamped));
    turnMotor.requestVoltage(Volts.of(turnVoltsClamped));

    inputs.driveConnected = true;
    inputs.drivePositionRad = moduleSimulation.getDriveWheelFinalPosition().in(Radians);
    inputs.driveVelocityRadPerSec = driveVelRadPerSec;
    inputs.driveAppliedVolts = driveVoltsClamped;
    inputs.driveCurrentAmps = Math.abs(moduleSimulation.getDriveMotorStatorCurrent().in(Amps));

    inputs.turnConnected = true;
    inputs.turnEncoderConnected = true;
    inputs.turnAbsolutePosition = moduleSimulation.getSteerAbsoluteFacing();
    inputs.turnPosition = inputs.turnAbsolutePosition;
    inputs.turnVelocityRadPerSec =
        moduleSimulation.getSteerAbsoluteEncoderSpeed().in(RadiansPerSecond);
    inputs.turnAppliedVolts = turnVoltsClamped;
    inputs.turnCurrentAmps = Math.abs(moduleSimulation.getSteerMotorStatorCurrent().in(Amps));

    inputs.odometryTimestamps = new double[] {Timer.getMonotonicTimestamp()};
    inputs.odometryDrivePositionsRad = new double[] {inputs.drivePositionRad};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveOpenLoop(double output) {
    driveClosedLoop = false;
    driveAppliedVolts = output;
  }

  @Override
  public void setDriveVelocity(double velocityRadPerSec) {
    driveClosedLoop = true;
    driveFFVolts = DRIVE_KS * Math.signum(velocityRadPerSec) + DRIVE_KV * velocityRadPerSec;
    driveController.setSetpoint(velocityRadPerSec);
  }

  @Override
  public void setTurnOpenLoop(double output) {
    turnClosedLoop = false;
    turnFFVolts = 0.0;
    turnAppliedVolts = output;
  }

  @Override
  public void setTurnPosition(Rotation2d rotation, double velocityRadPerSec) {
    turnClosedLoop = true;
    turnFFVolts = TURN_KV * velocityRadPerSec;
    turnController.setSetpoint(rotation.getRadians());
  }
}
