package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.wpilib.units.Units.RotationsPerSecond;
import static org.wpilib.units.Units.Volts;

import org.junit.jupiter.api.Test;
import org.wpilib.units.measure.AngularVelocity;

class MeasuresTest {
  @Test
  void finiteMeasuresPass() {
    assertTrue(Measures.isFinite(RotationsPerSecond.of(40.0)));
    assertTrue(Measures.isFinite(Volts.of(-8.0)));
    assertTrue(Measures.isFinite(Volts.of(0.0)));
  }

  @Test
  void nonFiniteMeasuresFail() {
    assertFalse(Measures.isFinite(null));
    assertFalse(Measures.isFinite(RotationsPerSecond.of(Double.NaN)));
    assertFalse(Measures.isFinite(Volts.of(Double.POSITIVE_INFINITY)));
    assertFalse(Measures.isFinite(Volts.of(Double.NEGATIVE_INFINITY)));
  }

  @Test
  void finiteVoltageNeutralizesBadInput() {
    assertEquals(0.0, Measures.finiteVoltage(null).in(Volts));
    assertEquals(0.0, Measures.finiteVoltage(Volts.of(Double.NaN)).in(Volts));
    assertEquals(6.0, Measures.finiteVoltage(Volts.of(6.0)).in(Volts));
  }

  @Test
  void anyNonFiniteAngularVelocityFails() {
    double[] bad = {Double.NaN, Double.POSITIVE_INFINITY, Double.NEGATIVE_INFINITY};
    for (double value : bad) {
      AngularVelocity velocity = RotationsPerSecond.of(value);
      assertFalse(Measures.isFinite(velocity), "expected non-finite: " + value);
    }
  }
}
