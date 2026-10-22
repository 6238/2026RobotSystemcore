package frc.robot.util;

import frc.robot.generated.COMPTunerConstants;
import frc.robot.generated.ConcreteMk4TunerConstants;
import frc.robot.generated.RobotTunerConstants;
import org.littletonrobotics.junction.Logger;

/**
 * Selects which generated tuner file the existing swerve uses. This project defaults to the
 * concrete-floor campaign.
 */
public class RobotIdentity {
  public enum RobotType {
    CONCRETE_MK4,
    COMP
  }

  /** Switch identities here. Drive/IO always read {@link #getTunerConstants()}. */
  public static final RobotType ACTIVE = RobotType.CONCRETE_MK4;

  public static RobotType getRobotType() {
    return ACTIVE;
  }

  private static RobotTunerConstants cached;

  public static RobotTunerConstants getTunerConstants() {
    if (cached != null) {
      return cached;
    }

    Logger.recordMetadata("Robot Type", getRobotType().toString());

    cached =
        switch (getRobotType()) {
          case CONCRETE_MK4 -> fromConcreteMk4();
          case COMP -> fromComp();
        };
    return cached;
  }

  private static RobotTunerConstants fromConcreteMk4() {
    return new RobotTunerConstants(
        ConcreteMk4TunerConstants.kCANBus,
        ConcreteMk4TunerConstants.DrivetrainConstants,
        ConcreteMk4TunerConstants.FrontLeft,
        ConcreteMk4TunerConstants.FrontRight,
        ConcreteMk4TunerConstants.BackLeft,
        ConcreteMk4TunerConstants.BackRight,
        ConcreteMk4TunerConstants.kSpeedAt12Volts);
  }

  private static RobotTunerConstants fromComp() {
    return new RobotTunerConstants(
        COMPTunerConstants.kCANBus,
        COMPTunerConstants.DrivetrainConstants,
        COMPTunerConstants.FrontLeft,
        COMPTunerConstants.FrontRight,
        COMPTunerConstants.BackLeft,
        COMPTunerConstants.BackRight,
        COMPTunerConstants.kSpeedAt12Volts);
  }
}
