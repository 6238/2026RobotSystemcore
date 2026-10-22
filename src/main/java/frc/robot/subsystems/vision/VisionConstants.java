package frc.robot.subsystems.vision;

import static org.wpilib.units.Units.Degrees;
import static org.wpilib.units.Units.Inches;
import static org.wpilib.units.Units.Radians;

import org.wpilib.fields.Field;
import org.wpilib.fields.Fields;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation3d;

public class VisionConstants {
  // AprilTag layout
  public static Field aprilTagLayout = Field.loadField(Fields.FRC_2026_REBUILT_WELDED);

  // Camera names, must match names configured on coprocessor
  public static String camera0Name = "FRONT";

  // Robot to camera transforms
  public static Transform3d robotToCamera0 =
      new Transform3d(
          new Translation3d(Inches.of(12.978 - 6), Inches.of(-8.173559), Inches.of(20)),
          new Rotation3d(
              Degrees.of(0).in(Radians),
              Degrees.of(-14).in(Radians),
              Degrees.of(-5.35).in(Radians)));

  public static String camera1Name = "SIDE";

  // Robot to camera transforms
  public static Transform3d robotToCamera1 =
      new Transform3d(
          new Translation3d(Inches.of(29.5 / 2 - 2), Inches.of(11), Inches.of(15)),
          new Rotation3d(
              Degrees.of(0).in(Radians), Degrees.of(0).in(Radians), Degrees.of(90).in(Radians)));

  // Basic filtering thresholds
  public static double maxAmbiguity = 0.14;
  public static double maxZError = 0.65;
  //   public static double maxTagDistance = 4; // Meters

  // X coordinate exclusion zones (robot pose X ranges to reject, e.g. field structures)
  public static double[][] xExclusionZones = new double[][] {{4.0, 5.292}, {11.5, 12.5}};

  // Standard deviation baselines, for 1 meter distance and 1 tag
  // (Adjusted automatically based on distance and # of tags)
  public static double linearStdDevBaseline = 0.07; // Meters
  public static double angularStdDevBaseline = 0.09; // Radians

  // Standard deviation multipliers for each camera
  // (Adjust to trust some cameras more than others)
  public static double[] cameraStdDevFactors =
      new double[] {
        1.0, // Camera 0
        1.0,
      };
}
