package frc.robot.util;

public final class SineGoalGenerator {
  private SineGoalGenerator() {}

  /**
   * Generates a sinusoidal goal in degrees.
   *
   * @param tSec time (seconds)
   * @param offsetDeg center offset (degrees)
   * @param amplitudeDeg amplitude (degrees, non-negative)
   * @param periodSec period (seconds, must be > 0)
   */
  public static double goalDeg(double tSec, double offsetDeg, double amplitudeDeg, double periodSec) {
    if (periodSec <= 0) return offsetDeg;
    double amp = Math.abs(amplitudeDeg);
    return offsetDeg + amp * Math.sin(2.0 * Math.PI * tSec / periodSec);
  }
}


