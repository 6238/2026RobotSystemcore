package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class LeaderFailoverTest {
  @Test
  void backupOnlyWhenLeadIsDownAndBackupIsUp() {
    boolean[] states = {false, true};
    for (boolean lead : states) {
      for (boolean backup : states) {
        assertEquals(!lead && backup, LeaderFailover.useBackup(lead, backup));
      }
    }
  }
}
