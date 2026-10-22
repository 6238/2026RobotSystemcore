package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.Radians;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.ExtrapolatingDoubleTreeMap;
import frc.robot.util.PhoenixUtil;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;

public final class ShooterConstants {
  private ShooterConstants() {}

  public static final int FLYWHEEL_MOTOR_ID = 39;
  public static final int FLYWHEEL_FOLLOWER_ID = 48;
  public static final int FEEDER_MOTOR_ID = 40;
  public static final int BEAM_BREAK_CHANNEL = 0;
  public static final boolean BEAM_BREAK_INVERTED = false;
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

  public static final InvertedValue FLYWHEEL_INVERTED = InvertedValue.Clockwise_Positive;
  public static final InvertedValue FEEDER_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final MotorAlignmentValue MOTOR_ALIGNMENT = MotorAlignmentValue.Opposed;

  public static final double FLYWHEEL_GEARING = 18.0 / 24.0;
  public static final double FEEDER_GEARING = 18.0 / 12.0;

  public static final double MAX_VOLTS = 12.0;
  public static final double FLYWHEEL_STATOR_CURRENT_LIMIT = 130.0;
  public static final double FLYWHEEL_SUPPLY_CURRENT_LIMIT = 60.0;
  public static final double FLYWHEEL_SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
  public static final double FEEDER_STATOR_CURRENT_LIMIT = 120.0;
  public static final double FEEDER_SUPPLY_CURRENT_LIMIT = 50.0;
  public static final double FEEDER_SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
  public static final double SUPPLY_CURRENT_LOWER_TIME_SECONDS = 0.5;
  public static final double FLYWHEEL_CONNECTION_DEBOUNCE_SECONDS = 0.5;

  public static final double FEEDER_REVERSE_VOLTS = 8.0;
  public static final double SHOOTING_WINDOW_SECONDS = 0.3;
  public static final boolean SPINUP_WHEN_HUB_ACTIVE = true;

  public static final AngularVelocity SPINUP_FLYWHEEL_SPEED = RotationsPerSecond.of(50.0);
  public static final AngularVelocity FEEDER_SPEED = RotationsPerSecond.of(50.0);
  public static final AngularVelocity FLYWHEEL_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(0.5);
  public static final AngularVelocity FEEDER_TOLERANCE_BEFORE_SHOT = RotationsPerSecond.of(2.0);
  public static final AngularVelocity FLYWHEEL_SPINUP_TRANSITION_TOLERANCE =
      RotationsPerSecond.of(2.0);
  public static final AngularVelocity BIG_FLYWHEEL_TOLERANCE_BEFORE_SHOT =
      RotationsPerSecond.of(0.8);
  public static final double PASSING_FLYWHEEL_MIN_RPS = 30.0;
  public static final double PASSING_FLYWHEEL_MAX_RPS = 100.0;

  public static final Distance SHOOTER_WHEEL_RADIUS = Inches.of(2.0);
  public static final Distance SHOOTER_HOOD_HEIGHT = Inches.of(28.0);
  public static final Angle FIXED_HOOD_ANGLE = Degrees.of(60.5);
  public static final double FLYWHEEL_ENERGY_TRANSFER_COEFF = 1.5;
  public static final double BALL_SPEED_PER_FLYWHEEL_RPS =
      2.0 * Math.PI * SHOOTER_WHEEL_RADIUS.in(Meters) / FLYWHEEL_ENERGY_TRANSFER_COEFF;

  public static final Angle HUB_ROTATION_TOLERANCE = Degrees.of(5.0);
  public static final Angle HUB_ROTATION_TOLERANCE_TIGHT = Degrees.of(3.0);
  public static final Angle HUB_DROPBACK_TOLERANCE = Degrees.of(9.0);
  public static final Angle HUB_PASSING_DROPBACK_TOLERANCE = Degrees.of(12.5);
  public static final Distance HUB_POSITION_TOLERANCE = Meters.of(0.04);
  public static final double MIN_SHOT_DISTANCE_METERS = 2.2;
  public static final double HUB_NEAR_DISTANCE_METERS = 3.0;
  public static final double HUB_HIGH_ROBOT_SPEED_MPS = 2.5;

  public static final Slot0Configs FLYWHEEL_SLOT_0 = flywheelSlot0();
  public static final Slot0Configs FEEDER_SLOT_0 = feederSlot0();

  public static final ExtrapolatingDoubleTreeMap FLYWHEEL_MAP = new ExtrapolatingDoubleTreeMap();
  public static final ExtrapolatingDoubleTreeMap FEEDER_MAP = new ExtrapolatingDoubleTreeMap();
  public static final ExtrapolatingDoubleTreeMap LEAD_TIME_MAP = new ExtrapolatingDoubleTreeMap();
  public static final ExtrapolatingDoubleTreeMap PASSING_FLYWHEEL_MAP =
      new ExtrapolatingDoubleTreeMap();
  public static final ExtrapolatingDoubleTreeMap PASSING_LEAD_TIME_MAP =
      new ExtrapolatingDoubleTreeMap();

  static {
    FLYWHEEL_MAP.put(2.32, 42.7);
    FLYWHEEL_MAP.put(2.53, 43.9);
    FLYWHEEL_MAP.put(3.52, 53.2 * 0.97);
    FLYWHEEL_MAP.put(4.14, 54.6 * 0.97);

    FEEDER_MAP.put(2.32, 43.5);
    FEEDER_MAP.put(2.53, 44.5);
    FEEDER_MAP.put(3.52, 52.0);
    FEEDER_MAP.put(4.14, 55.4);

    double cosHood = Math.cos(FIXED_HOOD_ANGLE.in(Radians));
    for (double distanceM = 0.5; distanceM <= 8.0; distanceM += 0.25) {
      double rps = FLYWHEEL_MAP.get(distanceM);
      double flightSec = distanceM / (rps * BALL_SPEED_PER_FLYWHEEL_RPS * cosHood);
      LEAD_TIME_MAP.put(distanceM, flightSec * (1.0 + distanceM * distanceM / 32.0));
    }

    PASSING_FLYWHEEL_MAP.put(1.5, 27.5 / 0.75);
    PASSING_FLYWHEEL_MAP.put(3.0, 34.75 / 0.75);
    for (double distanceM = 0.5; distanceM <= 8.0; distanceM += 0.25) {
      double rps = PASSING_FLYWHEEL_MAP.get(distanceM);
      double flightSec = distanceM / (rps * BALL_SPEED_PER_FLYWHEEL_RPS * cosHood);
      PASSING_LEAD_TIME_MAP.put(distanceM, flightSec);
    }
  }

  public static TalonFXConfiguration flywheelConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = FLYWHEEL_SLOT_0;
    config.Feedback.SensorToMechanismRatio = FLYWHEEL_GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = FLYWHEEL_INVERTED;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, FLYWHEEL_STATOR_CURRENT_LIMIT, FLYWHEEL_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = FLYWHEEL_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    return config;
  }

  public static TalonFXConfiguration feederConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = FEEDER_SLOT_0;
    config.Feedback.SensorToMechanismRatio = FEEDER_GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = FEEDER_INVERTED;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, FEEDER_STATOR_CURRENT_LIMIT, FEEDER_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = FEEDER_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    return config;
  }

  private static Slot0Configs flywheelSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 0.6;
    slot.kI = 0.0;
    slot.kD = 0.0;
    slot.kA = 0.172 / FLYWHEEL_GEARING;
    slot.kV = 0.0631 / FLYWHEEL_GEARING;
    slot.kS = 0.25 / FLYWHEEL_GEARING;
    slot.kG = 0.0;
    return slot;
  }

  private static Slot0Configs feederSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 0.6;
    slot.kI = 0.0;
    slot.kD = 0.0;
    // 2026 feeder FF was tuned against flywheel gearing; keep that so the slot matches the robot.
    slot.kA = 0.172 / FLYWHEEL_GEARING;
    slot.kV = 0.0831 / FLYWHEEL_GEARING;
    slot.kS = 0.35 / FLYWHEEL_GEARING;
    slot.kG = 0.0;
    return slot;
  }
}
