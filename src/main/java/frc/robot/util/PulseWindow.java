package frc.robot.util;

/** Latches the last time a pulse was true and reports whether it is still inside a window. */
public final class PulseWindow {
  private double lastTriggerSec = Double.NaN;

  public void update(boolean triggered, double nowSec) {
    if (triggered) {
      lastTriggerSec = nowSec;
    }
  }

  public boolean active(double nowSec, double windowSec) {
    return Double.isFinite(lastTriggerSec) && (nowSec - lastTriggerSec) < windowSec;
  }

  public double elapsedSec(double nowSec) {
    if (!Double.isFinite(lastTriggerSec)) {
      return Double.MAX_VALUE;
    }
    return nowSec - lastTriggerSec;
  }
}
