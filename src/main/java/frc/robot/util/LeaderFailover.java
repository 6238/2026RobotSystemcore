package frc.robot.util;

/** Prefer the lead device; command the backup only when the lead is down and the backup is up. */
public final class LeaderFailover {
  private LeaderFailover() {}

  public static boolean useBackup(boolean leadConnected, boolean backupConnected) {
    return !leadConnected && backupConnected;
  }
}
