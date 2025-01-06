package frc.robot.subsystems.piece_detection;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class PieceDetection extends SubsystemBase {
  private PieceDetectionIO pieceDetection;
  private PieceDetectionIOInputsAutoLogged inputs = new PieceDetectionIOInputsAutoLogged();

  private final String name;

  private final Supplier<Pose3d> poseSupplier;
  private Pose3d gamePiecePose;

  public PieceDetection(PieceDetectionIO io, Supplier<Pose3d> robotPoseSupplier) {
    pieceDetection = io;

    poseSupplier = robotPoseSupplier;

    name = pieceDetection.getName();
  }

  @Override
  public void periodic() {
    pieceDetection.updateInputs(inputs);

    gamePiecePose = poseSupplier.get().transformBy(inputs.pieceTransform);

    Logger.recordOutput(name + "/Pose", getGamePiecePose());

    Logger.processInputs(name, inputs);
  }

  public double getPitch() {
    return inputs.pitch;
  }

  public double getYaw() {
    return inputs.yaw;
  }

  public double getArea() {
    return inputs.area;
  }

  public Pose3d getGamePiecePose() {
    return gamePiecePose;
  }

  public boolean pieceDetected() {
    return inputs.seesTarget;
  }
}
