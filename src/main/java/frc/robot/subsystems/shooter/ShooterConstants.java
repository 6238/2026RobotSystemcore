package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.wpilib.hardware.bus.CANPort;

public class ShooterConstants {
  public static final int LEFT_TALON_ID = 0;
  public static final int RIGHT_TALON_ID = 1;
  public static final int FEEDER_TALON_ID = 2;
  public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

  public static final Slot0Configs FLYWHEEL_SLOT_0 = new Slot0Configs();
  public static final Slot0Configs FEEDER_SLOT_0 = new Slot0Configs();
  public static final MotorAlignmentValue MOTOR_ALLIGNMENT = MotorAlignmentValue.Opposed;
}
