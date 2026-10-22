package frc.robot.subsystems.shooter;

import static org.wpilib.units.Units.RotationsPerSecond;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import org.wpilib.command3.Mechanism;
import org.wpilib.math.util.MathUtil;
import org.wpilib.units.measure.AngularVelocity;

public class Shooter implements Mechanism {
    private final ShooterIO io;
    private final ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();
    private AngularVelocity targetFeederVelocity;
    private AngularVelocity targetFlywheelVelocity;

    private boolean isFlywheelIdle = true;
    private boolean isFeederIdle = false;

    public Shooter(ShooterIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter", inputs);

        Logger.recordOutput("Shooter/TargetFeederVelocity", targetFeederVelocity);
        Logger.recordOutput("Shooter/TargetFlywheelVelocity", targetFlywheelVelocity);
        Logger.recordOutput("Shooter/isFlywheelIdle", isFlywheelIdle);
        Logger.recordOutput("Shooter/isFeederIdle", isFeederIdle);
    }

    public Command runFlywheelVelocity(AngularVelocity velocity) {
        return run((c) -> {
            io.setFlywheelVelocity(velocity);
            targetFlywheelVelocity = velocity;
            isFlywheelIdle = false;
        }).named("ShooterRunFlywheelVelocity");
    }

    public Command runFeederVelocity(AngularVelocity velocity) {
        return run((c) -> {
            io.setFeederVelocity(velocity);
            targetFeederVelocity = velocity;
            isFeederIdle = false;
        }).named("ShooterRunFeederVelocity");
    }

    public Command runIdle() {
        return run((c) -> {
            io.idleFlywheel();
            io.idleFeeder();
            isFlywheelIdle = true;
            isFeederIdle = true;
        }).named("ShooterRunIdle");
    }

    public boolean atSetpoints(double tolerance) {
        return MathUtil.isNear(
            targetFeederVelocity.in(RotationsPerSecond),
            inputs.feederVelocity.in(RotationsPerSecond),
            tolerance
        ) && MathUtil.isNear(
            targetFlywheelVelocity.in(RotationsPerSecond),
            inputs.shooterVelocity.in(RotationsPerSecond),
            tolerance
        );
    }
}
