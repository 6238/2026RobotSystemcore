package frc.robot.subsystems.hopper;

import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import frc.robot.util.Measures;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.util.Alert;
import org.wpilib.util.Alert.Level;

/** Bottom and top indexers. One mechanism because Superstructure states always command both. */
public class Hopper implements Mechanism, AutoCloseable {
  private final HopperIO io;
  private final HopperInputsAutoLogged inputs = new HopperInputsAutoLogged();

  private AngularVelocity indexerSetpoint;
  private AngularVelocity topIndexerSetpoint;

  private final Alert indexerDisconnectedAlert =
      new Alert("Hopper", "indexerDisconnected", "Hopper indexer motor disconnected.", Level.HIGH);
  private final Alert topIndexerDisconnectedAlert =
      new Alert(
          "Hopper", "topIndexerDisconnected", "Hopper top indexer motor disconnected.", Level.HIGH);

  public Hopper(HopperIO io) {
    this.io = io;
    setDefaultCommand(idle());
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Hopper", inputs);

    indexerDisconnectedAlert.set(!inputs.indexerTalonConnected);
    topIndexerDisconnectedAlert.set(!inputs.topIndexerTalonConnected);

    Logger.recordOutput("Hopper/IndexerSetpoint", setpointOrZero(indexerSetpoint));
    Logger.recordOutput("Hopper/TopIndexerSetpoint", setpointOrZero(topIndexerSetpoint));
    Logger.recordOutput("Hopper/IndexerClosedLoop", indexerSetpoint != null);
    Logger.recordOutput("Hopper/TopIndexerClosedLoop", topIndexerSetpoint != null);
  }

  @Override
  public Command idle() {
    return runRepeatedly(
            () -> {
              indexerSetpoint = null;
              topIndexerSetpoint = null;
              io.setIndexerVoltage(Volts.of(0.0));
              io.setTopIndexerVoltage(Volts.of(0.0));
            })
        .withPriority(Command.LOWEST_PRIORITY)
        .named("Hopper[IDLE]");
  }

  public Command feed() {
    return feed(() -> HopperConstants.INDEXER_SPEED, () -> HopperConstants.TOP_INDEXER_SPEED);
  }

  public Command feed(AngularVelocity indexer, AngularVelocity topIndexer) {
    return feed(() -> indexer, () -> topIndexer);
  }

  public Command feed(Supplier<AngularVelocity> indexer, Supplier<AngularVelocity> topIndexer) {
    return runRepeatedly(
            () -> {
              indexerSetpoint = finiteOrNull(indexer.get());
              topIndexerSetpoint = finiteOrNull(topIndexer.get());
              if (indexerSetpoint != null) {
                io.setIndexerVelocity(indexerSetpoint);
              } else {
                io.setIndexerVoltage(Volts.of(0.0));
              }
              if (topIndexerSetpoint != null) {
                io.setTopIndexerVelocity(topIndexerSetpoint);
              } else {
                io.setTopIndexerVoltage(Volts.of(0.0));
              }
            })
        .named("HopperFeed");
  }

  public Command reverse() {
    return runRepeatedly(
            () -> {
              indexerSetpoint = HopperConstants.INDEXER_SPEED.unaryMinus();
              topIndexerSetpoint = HopperConstants.TOP_INDEXER_SPEED.unaryMinus();
              io.setIndexerVelocity(indexerSetpoint);
              io.setTopIndexerVelocity(topIndexerSetpoint);
            })
        .named("HopperReverse");
  }

  public boolean indexerAtSpeed(AngularVelocity tolerance) {
    return indexerSetpoint != null && inputs.indexerVelocity.isNear(indexerSetpoint, tolerance);
  }

  public boolean topIndexerAtSpeed(AngularVelocity tolerance) {
    return topIndexerSetpoint != null
        && inputs.topIndexerVelocity.isNear(topIndexerSetpoint, tolerance);
  }

  @Override
  public void close() {
    indexerDisconnectedAlert.close();
    topIndexerDisconnectedAlert.close();
  }

  private static AngularVelocity finiteOrNull(AngularVelocity velocity) {
    return Measures.isFinite(velocity) ? velocity : null;
  }

  private static AngularVelocity setpointOrZero(AngularVelocity setpoint) {
    return setpoint != null ? setpoint : RotationsPerSecond.of(0.0);
  }
}
