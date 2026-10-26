package frc.robot.util;

/**
 * Sticky leader/follower failover. Stay on the current leader. Switch only when that motor is down
 * and the other motor is up.
 */
public final class LeaderFailover {
  private LeaderFailover() {}

  public static boolean nextUsingBackup(
      boolean usingBackup, boolean leadConnected, boolean backupConnected) {
    boolean currentConnected = usingBackup ? backupConnected : leadConnected;
    boolean otherConnected = usingBackup ? leadConnected : backupConnected;
    if (!currentConnected && otherConnected) {
      return !usingBackup;
    }
    return usingBackup;
  }
}
