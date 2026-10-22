package frc.robot;

import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.framework.RobotBase;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final double ROBOT_PERIOD = 0.01;

  public static final Mode simMode = Mode.SIM;
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : simMode;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static boolean allianceIsRed() {
    return MatchState.getAlliance().isPresent() && MatchState.getAlliance().get() == Alliance.RED;
  }

  public static Pose2d hubPose() {
    return allianceIsRed()
        ? new Pose2d(11.932, 4.147, Rotation2d.ZERO)
        : new Pose2d(4.625, 4.045, Rotation2d.ZERO);
  }

  public static Pose2d leftPassPose() {
    return allianceIsRed()
        ? new Pose2d(14.0, 5.75, Rotation2d.ZERO)
        : new Pose2d(1.75, 5.75, Rotation2d.ZERO);
  }

  public static Pose2d rightPassPose() {
    return allianceIsRed()
        ? new Pose2d(14.0, 2.0, Rotation2d.ZERO)
        : new Pose2d(1.75, 2.0, Rotation2d.ZERO);
  }

  public static Pose2d passPose(Pose2d robot) {
    return robot.getY() > hubPose().getY() ? leftPassPose() : rightPassPose();
  }

  public static boolean shouldPass(double poseX) {
    return allianceIsRed() ? poseX < 11.5 : poseX > 4.75;
  }
}
