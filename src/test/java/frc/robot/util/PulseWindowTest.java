package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class PulseWindowTest {
  @Test
  void inactiveUntilAPulse() {
    PulseWindow window = new PulseWindow();
    assertFalse(window.active(0.0, 0.3));
    assertEquals(Double.MAX_VALUE, window.elapsedSec(0.0));
  }

  @Test
  void pulseOpensThenCloses() {
    PulseWindow window = new PulseWindow();
    window.update(true, 1.0);
    assertTrue(window.active(1.0, 0.3));
    window.update(false, 1.2);
    assertTrue(window.active(1.2, 0.3));
    assertFalse(window.active(1.31, 0.3));
  }
}
