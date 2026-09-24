package frc.robot.subsystems.drive;

import java.util.Optional;

/**
 * Least-squares fit of {@code V = kS·sign(v) + kV·v + kA·a}. Velocity and acceleration must share
 * one unit; the gains come back in volts per that unit. This is the chokepoint the drive
 * characterization command and the calibration tests both call.
 */
public record DriveFeedforwardFit(double kS, double kV, double kA) {
  public record Sample(double voltage, double velocity, double acceleration) {}

  /** Fits kS and kV, forcing kA to zero. Used when acceleration barely changes. */
  public static Optional<DriveFeedforwardFit> fitVelocity(Sample[] samples) {
    if (samples == null || samples.length < 2) {
      return Optional.empty();
    }
    double[][] normal = new double[2][2];
    double[] rhs = new double[2];
    int used = 0;
    for (Sample sample : samples) {
      if (sample == null || !Double.isFinite(sample.voltage) || !Double.isFinite(sample.velocity)) {
        continue;
      }
      double sign = Math.signum(sample.velocity);
      if (sign == 0.0) {
        sign = Math.signum(sample.voltage);
      }
      double[] row = {sign, sample.velocity};
      for (int r = 0; r < 2; r++) {
        rhs[r] += row[r] * sample.voltage;
        for (int c = 0; c < 2; c++) {
          normal[r][c] += row[r] * row[c];
        }
      }
      used++;
    }
    if (used < 2 || Math.abs(normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0]) < 1e-9) {
      return Optional.empty();
    }
    double det = normal[0][0] * normal[1][1] - normal[0][1] * normal[1][0];
    double kS = (rhs[0] * normal[1][1] - normal[0][1] * rhs[1]) / det;
    double kV = (normal[0][0] * rhs[1] - rhs[0] * normal[1][0]) / det;
    return Optional.of(new DriveFeedforwardFit(kS, kV, 0.0));
  }

  /** Fits kS, kV, and kA. Empty when the samples do not span all three columns. */
  public static Optional<DriveFeedforwardFit> fit(Sample[] samples) {
    if (samples == null || samples.length < 3) {
      return Optional.empty();
    }
    double[][] normal = new double[3][3];
    double[] rhs = new double[3];
    int used = 0;
    for (Sample sample : samples) {
      if (sample == null
          || !Double.isFinite(sample.voltage)
          || !Double.isFinite(sample.velocity)
          || !Double.isFinite(sample.acceleration)) {
        continue;
      }
      double sign = Math.signum(sample.velocity);
      if (sign == 0.0) {
        sign = Math.signum(sample.voltage);
      }
      double[] row = {sign, sample.velocity, sample.acceleration};
      for (int r = 0; r < 3; r++) {
        rhs[r] += row[r] * sample.voltage;
        for (int c = 0; c < 3; c++) {
          normal[r][c] += row[r] * row[c];
        }
      }
      used++;
    }
    if (used < 3) {
      return Optional.empty();
    }
    double[] solution = solve(normal, rhs);
    if (solution == null) {
      return Optional.empty();
    }
    return Optional.of(new DriveFeedforwardFit(solution[0], solution[1], solution[2]));
  }

  private static double[] solve(double[][] matrix, double[] rhs) {
    double[][] a = new double[3][4];
    for (int r = 0; r < 3; r++) {
      System.arraycopy(matrix[r], 0, a[r], 0, 3);
      a[r][3] = rhs[r];
    }
    for (int col = 0; col < 3; col++) {
      int pivot = col;
      for (int r = col + 1; r < 3; r++) {
        if (Math.abs(a[r][col]) > Math.abs(a[pivot][col])) {
          pivot = r;
        }
      }
      if (Math.abs(a[pivot][col]) < 1e-9) {
        return null;
      }
      double[] swap = a[col];
      a[col] = a[pivot];
      a[pivot] = swap;
      double scale = a[col][col];
      for (int c = col; c < 4; c++) {
        a[col][c] /= scale;
      }
      for (int r = 0; r < 3; r++) {
        if (r == col) {
          continue;
        }
        double factor = a[r][col];
        for (int c = col; c < 4; c++) {
          a[r][c] -= factor * a[col][c];
        }
      }
    }
    return new double[] {a[0][3], a[1][3], a[2][3]};
  }
}
