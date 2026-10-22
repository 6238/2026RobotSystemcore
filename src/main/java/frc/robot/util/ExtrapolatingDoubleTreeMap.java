package frc.robot.util;

import java.util.TreeMap;

/**
 * Interpolates between stored points and linearly extrapolates past the endpoints using the slope
 * of the two nearest entries.
 */
public class ExtrapolatingDoubleTreeMap {
  private final TreeMap<Double, Double> map = new TreeMap<>();

  public void put(double key, double value) {
    map.put(key, value);
  }

  public double get(double key) {
    if (map.isEmpty()) {
      return 0.0;
    }

    Double exactMatch = map.get(key);
    if (exactMatch != null) {
      return exactMatch;
    }

    if (map.size() == 1) {
      return map.firstEntry().getValue();
    }

    Double lowerKey = map.floorKey(key);
    Double upperKey = map.ceilingKey(key);

    if (lowerKey == null) {
      double k0 = map.firstKey();
      double k1 = map.higherKey(k0);
      return interpolate(k0, map.get(k0), k1, map.get(k1), key);
    }

    if (upperKey == null) {
      double k1 = map.lastKey();
      double k0 = map.lowerKey(k1);
      return interpolate(k0, map.get(k0), k1, map.get(k1), key);
    }

    return interpolate(lowerKey, map.get(lowerKey), upperKey, map.get(upperKey), key);
  }

  private static double interpolate(double x0, double y0, double x1, double y1, double x) {
    if (x0 == x1) {
      return y0;
    }
    double t = (x - x0) / (x1 - x0);
    return y0 + t * (y1 - y0);
  }
}
