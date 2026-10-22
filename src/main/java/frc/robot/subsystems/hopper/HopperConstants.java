package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.AngularVelocity;

public final class HopperConstants {
  private HopperConstants() {}

  public static final int INDEXER_MOTOR_ID = 41;
  public static final int TOP_INDEXER_MOTOR_ID = 46;
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

  public static final InvertedValue INDEXER_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue TOP_INDEXER_INVERTED = InvertedValue.CounterClockwise_Positive;

  public static final double INDEXER_GEARING = 1.0;
  public static final double TOP_INDEXER_GEARING = 1.0;

  public static final double MAX_VOLTS = 12.0;
  public static final double INDEXER_STATOR_CURRENT_LIMIT = 40.0;
  public static final double INDEXER_SUPPLY_CURRENT_LIMIT = 20.0;
  public static final double INDEXER_SUPPLY_CURRENT_LOWER_LIMIT = 15.0;
  public static final double TOP_INDEXER_STATOR_CURRENT_LIMIT = 40.0;
  public static final double TOP_INDEXER_SUPPLY_CURRENT_LIMIT = 25.0;
  public static final double TOP_INDEXER_SUPPLY_CURRENT_LOWER_LIMIT = 20.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SECONDS = 0.35;

  public static final AngularVelocity INDEXER_SPEED = RotationsPerSecond.of(65.0);
  public static final AngularVelocity TOP_INDEXER_SPEED = RotationsPerSecond.of(65.0);
  public static final AngularVelocity SPINUP_REVERSE_SPEED = RotationsPerSecond.of(15.0);
  public static final AngularVelocity TOLERANCE = RotationsPerSecond.of(10.0);

  public static final double ON_MOVE_SCALE = 0.5;

  public static TalonFXConfiguration indexerConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = indexerSlot0();
    config.Feedback.SensorToMechanismRatio = INDEXER_GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = INDEXER_INVERTED;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, INDEXER_STATOR_CURRENT_LIMIT, INDEXER_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = INDEXER_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    return config;
  }

  public static TalonFXConfiguration topIndexerConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = topIndexerSlot0();
    config.Feedback.SensorToMechanismRatio = TOP_INDEXER_GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = TOP_INDEXER_INVERTED;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, TOP_INDEXER_STATOR_CURRENT_LIMIT, TOP_INDEXER_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = TOP_INDEXER_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    return config;
  }

  private static Slot0Configs indexerSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 0.6;
    slot.kI = 0.0;
    slot.kD = 0.0;
    slot.kA = 0.0;
    slot.kV = 0.10;
    slot.kS = 0.24;
    slot.kG = 0.0;
    return slot;
  }

  private static Slot0Configs topIndexerSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 0.4;
    slot.kI = 0.0;
    slot.kD = 0.0;
    slot.kA = 0.08;
    slot.kV = 0.10;
    slot.kS = 0.24;
    slot.kG = 0.0;
    return slot;
  }
}
