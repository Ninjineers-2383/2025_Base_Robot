package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class PieceDetectionConstants {
  // Leaving this here just in case
  public record PieceDetectionConfig(Pose3d cameraPose) {}

  public static PieceDetectionConfig CORAL_DETECTION =
      new PieceDetectionConfig(
          new Pose3d(
              new Translation3d(0, 0, Units.inchesToMeters(9.75)),
              new Rotation3d(0.0, Math.toRadians(20.0), 0.0)));
}
