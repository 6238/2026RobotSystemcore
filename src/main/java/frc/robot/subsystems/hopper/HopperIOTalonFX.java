package frc.robot.subsystems.hopper;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.StatusSignalCollection;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Voltage;

public class HopperIOTalonFX implements HopperIO {
  private final TalonFX indexerTalon;
  private final TalonFX topIndexerTalon;

  private final VelocityVoltage indexerVelocity = new VelocityVoltage(0);
  private final VelocityVoltage topIndexerVelocity = new VelocityVoltage(0);
  private final VoltageOut indexerVoltage = new VoltageOut(0);
  private final VoltageOut topIndexerVoltage = new VoltageOut(0);

  private final StatusSignalCollection signals;
  private final StatusSignal<AngularVelocity> indexerAngularVelocity;
  private final StatusSignal<Current> indexerSupplyCurrent;
  private final StatusSignal<Voltage> indexerMotorVoltage;
  private final StatusSignal<AngularVelocity> topIndexerAngularVelocity;
  private final StatusSignal<Current> topIndexerSupplyCurrent;
  private final StatusSignal<Voltage> topIndexerMotorVoltage;

  public HopperIOTalonFX() {
    indexerTalon = new TalonFX(HopperConstants.INDEXER_MOTOR_ID, HopperConstants.CAN_BUS);
    topIndexerTalon = new TalonFX(HopperConstants.TOP_INDEXER_MOTOR_ID, HopperConstants.CAN_BUS);

    tryUntilOk(
        5, () -> indexerTalon.getConfigurator().apply(HopperConstants.indexerConfig(), 0.25));
    tryUntilOk(
        5, () -> topIndexerTalon.getConfigurator().apply(HopperConstants.topIndexerConfig(), 0.25));

    indexerAngularVelocity = indexerTalon.getVelocity();
    indexerSupplyCurrent = indexerTalon.getSupplyCurrent();
    indexerMotorVoltage = indexerTalon.getMotorVoltage();
    topIndexerAngularVelocity = topIndexerTalon.getVelocity();
    topIndexerSupplyCurrent = topIndexerTalon.getSupplyCurrent();
    topIndexerMotorVoltage = topIndexerTalon.getMotorVoltage();

    signals =
        new StatusSignalCollection(
            indexerAngularVelocity,
            indexerSupplyCurrent,
            indexerMotorVoltage,
            topIndexerAngularVelocity,
            topIndexerSupplyCurrent,
            topIndexerMotorVoltage);
  }

  @Override
  public void updateInputs(HopperInputs inputs) {
    signals.refreshAll();

    inputs.indexerTalonConnected = indexerTalon.isConnected();
    inputs.indexerVelocity = indexerAngularVelocity.getValue();
    inputs.indexerSupplyCurrent = indexerSupplyCurrent.getValue();
    inputs.indexerAppliedVoltage = indexerMotorVoltage.getValue();

    inputs.topIndexerTalonConnected = topIndexerTalon.isConnected();
    inputs.topIndexerVelocity = topIndexerAngularVelocity.getValue();
    inputs.topIndexerSupplyCurrent = topIndexerSupplyCurrent.getValue();
    inputs.topIndexerAppliedVoltage = topIndexerMotorVoltage.getValue();
  }

  @Override
  public void setIndexerVelocity(AngularVelocity targetVelocity) {
    indexerTalon.setControl(indexerVelocity.withVelocity(targetVelocity));
  }

  @Override
  public void setIndexerVoltage(Voltage voltage) {
    indexerTalon.setControl(indexerVoltage.withOutput(voltage));
  }

  @Override
  public void setTopIndexerVelocity(AngularVelocity targetVelocity) {
    topIndexerTalon.setControl(topIndexerVelocity.withVelocity(targetVelocity));
  }

  @Override
  public void setTopIndexerVoltage(Voltage voltage) {
    topIndexerTalon.setControl(topIndexerVoltage.withOutput(voltage));
  }
}
