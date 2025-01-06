package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.piece_detection.PieceDetectionConstants.PieceDetectionConfig;
import org.photonvision.PhotonCamera;

public class PieceDetectionIOPhoton implements PieceDetectionIO {
  private final String name;

  private final PieceDetectionConfig config;
  private final PhotonCamera camera;

  private final Alert cameraDisconnected;

  private final NetworkTable photonTable;

  private final BooleanSubscriber seesTarget;

  private final DoubleSubscriber yaw;
  private final DoubleSubscriber pitch;

  public PieceDetectionIOPhoton(String name, PieceDetectionConfig config) {
    this.name = name;
    this.config = config;

    camera = new PhotonCamera(name);

    cameraDisconnected = new Alert(name, name + " disconnected!", AlertType.kWarning);

    photonTable = NetworkTableInstance.getDefault().getTable("photonvision/" + name);

    seesTarget =
        photonTable
            .getBooleanTopic("hasTarget")
            .subscribe(false, PubSubOption.keepDuplicates(false));

    yaw =
        photonTable.getDoubleTopic("targetYaw").subscribe(0.0, PubSubOption.keepDuplicates(false));

    pitch =
        photonTable
            .getDoubleTopic("targetPitch")
            .subscribe(0.0, PubSubOption.keepDuplicates(false));
  }

  @Override
  public void updateInputs(PieceDetectionIOInputs inputs) {
    inputs.connected = camera.isConnected();

    if (seesTarget.get()) {
      inputs.seesTarget = true;
      inputs.pitch = pitch.get();
      inputs.yaw = yaw.get();

      double pieceX =
          config.cameraPose().getZ()
              * Math.tan(
                  Math.toRadians(inputs.pitch)
                      + ((Math.PI / 2.0) - config.cameraPose().getRotation().getY()));
      inputs.pieceTransform =
          new Transform3d(
              new Translation3d(pieceX, pieceX * Math.tan(Math.toRadians(inputs.yaw)), 0.0),
              new Rotation3d());
    }

    cameraDisconnected.set(!inputs.connected);
  }

  @Override
  public String getName() {
    return name;
  }
}
