package frc.robot.subsystems.intake;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Rotations;
import static org.wpilib.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import frc.robot.util.PhoenixUtil;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;

public final class IntakeConstants {
  private IntakeConstants() {}

  public static final int PIVOT_MOTOR_ID = 54;
  public static final int PIVOT_CANCODER_ID = 56;
  public static final int ROLLER_MOTOR_ID = 59;
  public static final int ROLLER_FOLLOWER_ID = 55;

  public static final CANBus PIVOT_CAN_BUS = new CANBus(CANPort.CAN_S0);
  public static final CANBus ROLLER_CAN_BUS = new CANBus(CANPort.CAN_S1);

  public static final InvertedValue PIVOT_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final InvertedValue ROLLER_INVERTED = InvertedValue.CounterClockwise_Positive;
  public static final MotorAlignmentValue ROLLER_ALIGNMENT = MotorAlignmentValue.Opposed;

  public static final double PIVOT_GEARING = 35.0 * 2.0;
  public static final double ROLLER_GEARING = 1.5;
  public static final double CANCODER_MAGNET_OFFSET = -0.16;

  public static final Angle DOWN_ANGLE = Degrees.of(7.5);
  public static final Angle UP_ANGLE = Degrees.of(85.0);
  public static final Angle READY_ANGLE = Degrees.of(30.0);
  public static final Angle START_ANGLE = Degrees.of(120.0);

  public static final AngularVelocity ROLLER_SPEED = RotationsPerSecond.of(70.0);
  public static final double ROLLER_CUTOFF_DEGREES = 80.0;

  public static final double MAX_VOLTS = 12.0;
  public static final double PIVOT_STATOR_CURRENT_LIMIT = 60.0;
  public static final double PIVOT_SUPPLY_CURRENT_LIMIT = 30.0;
  public static final double PIVOT_SUPPLY_CURRENT_LOWER_LIMIT = 25.0;
  public static final double ROLLER_STATOR_CURRENT_LIMIT = 80.0;
  public static final double ROLLER_SUPPLY_CURRENT_LIMIT = 50.0;
  public static final double ROLLER_SUPPLY_CURRENT_LOWER_LIMIT = 40.0;
  public static final double PIVOT_SUPPLY_CURRENT_LOWER_TIME_SECONDS = 0.5;
  public static final double ROLLER_SUPPLY_CURRENT_LOWER_TIME_SECONDS = 0.35;
  public static final double ROLLER_CONNECTION_DEBOUNCE_SECONDS = 0.5;

  public static final double STALL_CURRENT_THRESHOLD_AMPS = 35.0;
  public static final double STALL_VELOCITY_THRESHOLD_RPS = 8.0;
  public static final double STALL_DEBOUNCE_SECONDS = 0.25;
  public static final double JAM_REVERSE_MIN_DURATION_SECONDS = 0.1875;
  public static final double JAM_REVERSE_VOLTS = 12.0;

  public static final double PRELOAD_VOLTAGE_VOLTS = 2.0;
  public static final double PRELOAD_CURRENT_THRESHOLD_AMPS = 10.0;
  public static final double PRELOAD_TIMEOUT_SECONDS = 1.0;

  public static final double OSCILLATE_DOWN_OFFSET_DEGREES = 30.0;
  public static final double OSCILLATE_CENTER_OFFSET_DEGREES = 30.0;
  public static final double OSCILLATE_AMPLITUDE_DEGREES = 10.0;
  public static final double OSCILLATE_FREQUENCY_HZ = 4.0;
  public static final double OSCILLATE_SWEEP_RATE_DPS = 12.0;
  public static final double OSCILLATE_DUTY_CYCLE = 0.7;

  public static TalonFXConfiguration pivotConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = pivotSlot0();
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Feedback.RotorToSensorRatio = -PIVOT_GEARING;
    config.Feedback.FeedbackRemoteSensorID = PIVOT_CANCODER_ID;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = PIVOT_INVERTED;
    config.MotionMagic.MotionMagicCruiseVelocity = 1.5;
    config.MotionMagic.MotionMagicAcceleration = 3.0;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, PIVOT_STATOR_CURRENT_LIMIT, PIVOT_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = PIVOT_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = PIVOT_SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = START_ANGLE.in(Rotations);
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = DOWN_ANGLE.in(Rotations);
    return config;
  }

  public static CANcoderConfiguration cancoderConfig() {
    var config = new CANcoderConfiguration();
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    config.MagnetSensor.MagnetOffset = CANCODER_MAGNET_OFFSET;
    return config;
  }

  public static TalonFXConfiguration rollerConfig() {
    var config = new TalonFXConfiguration();
    config.Slot0 = rollerSlot0();
    config.Feedback.SensorToMechanismRatio = ROLLER_GEARING;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = ROLLER_INVERTED;
    PhoenixUtil.applySafetyLimits(
        config, MAX_VOLTS, ROLLER_STATOR_CURRENT_LIMIT, ROLLER_SUPPLY_CURRENT_LIMIT);
    config.CurrentLimits.SupplyCurrentLowerLimit = ROLLER_SUPPLY_CURRENT_LOWER_LIMIT;
    config.CurrentLimits.SupplyCurrentLowerTime = ROLLER_SUPPLY_CURRENT_LOWER_TIME_SECONDS;
    return config;
  }

  private static Slot0Configs pivotSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 19.0;
    slot.kI = 2.0;
    slot.kD = 6.0;
    slot.kA = 0.16;
    slot.kV = 6.5;
    slot.kS = 0.0;
    slot.kG = 0.3;
    return slot;
  }

  private static Slot0Configs rollerSlot0() {
    var slot = new Slot0Configs();
    slot.kP = 0.01;
    slot.kI = 0.0;
    slot.kD = 0.0;
    slot.kA = 0.12;
    slot.kV = 0.17;
    slot.kS = 0.0;
    slot.kG = 0.2;
    return slot;
  }
}
