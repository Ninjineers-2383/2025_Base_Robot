package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.FloatArraySubscriber;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.TimestampedDouble;
import edu.wpi.first.networktables.TimestampedFloatArray;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import org.littletonrobotics.junction.Logger;

/** IO implementation for real Limelight hardware. */
public class VisionIOQuestNav implements VisionIO {
  public record QuestNavData(
      Pose3d pose,
      double batteryPercent,
      double timestamp,
      float[] translation,
      float[] rotation) {}

  // Configure Network Tables topics (questnav/...) to communicate with the Quest HMD
  private NetworkTableInstance nt4Instance = NetworkTableInstance.getDefault();
  private NetworkTable nt4Table = nt4Instance.getTable("questnav");
  private IntegerSubscriber questMiso = nt4Table.getIntegerTopic("miso").subscribe(0);
  private IntegerPublisher questMosi =
      nt4Table
          .getIntegerTopic("mosi")
          .publish(PubSubOption.keepDuplicates(true), PubSubOption.sendAll(true));

  // Subscribe to the Network Tables questnav data topics
  private DoubleSubscriber questTimestamp = nt4Table.getDoubleTopic("timestamp").subscribe(0.0f);
  private FloatArraySubscriber questPosition =
      nt4Table.getFloatArrayTopic("position").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private FloatArraySubscriber questAngles =
      nt4Table.getFloatArrayTopic("eulerAngles").subscribe(new float[] {0.0f, 0.0f, 0.0f});
  private DoubleSubscriber questBatteryPercent =
      nt4Table.getDoubleTopic("device/batteryPercent").subscribe(0.0f);

  /** Subscriber for heartbeat requests */
  private final DoubleSubscriber heartbeatRequestSub =
      nt4Table.getDoubleTopic("heartbeat/quest_to_robot").subscribe(0.0);
  /** Publisher for heartbeat responses */
  private final DoublePublisher heartbeatResponsePub =
      nt4Table.getDoubleTopic("heartbeat/robot_to_quest").publish();
  /** Last processed heartbeat request ID */
  private double lastProcessedHeartbeatId = 0;

  private final Transform3d robotToCamera;

  private final VisionIO absoluteVisionIO;
  private final VisionIOInputsAutoLogged absoluteInputs = new VisionIOInputsAutoLogged();

  private Translation3d[] questNavRawToFieldCoordinateSystemQueue = new Translation3d[15];
  private Translation3d questNavRawToFieldCoordinateSystem = new Translation3d();

  int count = 0;
  int idx = 0;

  protected Rotation3d gyroResetAngle = new Rotation3d();
  protected Pose3d lastPose3d = new Pose3d();

  public VisionIOQuestNav(Transform3d robotToCamera, VisionIO absoluteVisionIO) {
    // Initialize the camera to robot transform
    this.robotToCamera = robotToCamera;
    this.absoluteVisionIO = absoluteVisionIO;
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    QuestNavData[] questNavData = getQuestNavData();

    // Update the absolute vision IO
    absoluteVisionIO.updateInputs(absoluteInputs);
    Logger.processInputs("QuestNav/absolute", absoluteInputs);

    inputs.connected = connected();
    inputs.latestTargetObservation = new TargetObservation(new Rotation2d(), new Rotation2d());
    inputs.poseObservations = new PoseObservation[questNavData.length];

    if (absoluteInputs.poseObservations.length > 0 && questNavData.length > 0) {
      questNavRawToFieldCoordinateSystemQueue[idx] =
          absoluteInputs
              .poseObservations[0]
              .pose()
              .getTranslation()
              .minus(questNavData[0].pose.getTranslation().rotateBy(gyroResetAngle));
      count += 1;
      idx += 1;
      if (idx == questNavRawToFieldCoordinateSystemQueue.length) {
        idx = 0;
      }
      questNavRawToFieldCoordinateSystem = new Translation3d();
      for (int i = 0; i < Math.min(count, questNavRawToFieldCoordinateSystemQueue.length); i++) {
        questNavRawToFieldCoordinateSystem =
            questNavRawToFieldCoordinateSystem.plus(questNavRawToFieldCoordinateSystemQueue[i]);
      }
      questNavRawToFieldCoordinateSystem =
          questNavRawToFieldCoordinateSystem.div(
              Math.min(count, questNavRawToFieldCoordinateSystemQueue.length));
      Logger.recordOutput("QuestNav/RawToField", questNavRawToFieldCoordinateSystem);
    }

    for (int i = 0; i < questNavData.length; i++) {
      inputs.poseObservations[i] =
          new PoseObservation(
              questNavData[i].timestamp(),
              new Pose3d(
                  questNavData[i]
                      .pose()
                      .getTranslation()
                      .rotateBy(gyroResetAngle)
                      .plus(questNavRawToFieldCoordinateSystem),
                  questNavData[i].pose().getRotation().plus(gyroResetAngle)),
              0.0,
              -1,
              0.0,
              PoseObservationType.QUESTNAV);

      lastPose3d = inputs.poseObservations[i].pose();
    }
    inputs.tagIds = new int[0];

    Logger.recordOutput("QuestNav/battery", getBatteryPercent());

    cleanUpQuestNavMessages();
    processHeartbeat();
  }

  private QuestNavData[] getQuestNavData() {
    TimestampedDouble[] timestamps = questTimestamp.readQueue();
    TimestampedFloatArray[] positions = questPosition.readQueue();
    TimestampedFloatArray[] angles = questAngles.readQueue();
    // TimestampedDouble[] battery = questBatteryPercent.readQueue();

    double battery = getBatteryPercent();

    int length = Math.min(timestamps.length, Math.min(positions.length, angles.length));

    QuestNavData[] data = new QuestNavData[length];

    for (int i = 0; i < length; i++) {
      data[i] =
          new QuestNavData(
              getQuestNavPose(positions[i].value, angles[i].value).plus(robotToCamera.inverse()),
              battery,
              timestamps[i].timestamp,
              positions[i].value,
              angles[i].value);
    }

    return data;
  }

  // Gets the battery percent of the Quest.
  private double getBatteryPercent() {
    return questBatteryPercent.get();
  }

  // Returns if the Quest is connected.
  private boolean connected() {
    return ((RobotController.getFPGATime() - questBatteryPercent.getLastChange()) / 1000) < 250;
  }

  // Clean up questnav subroutine messages after processing on the headset
  private void cleanUpQuestNavMessages() {
    if (questMiso.get() == 99) {
      questMosi.set(0);
    }
  }

  private Translation3d getQuestNavTranslation(float[] position) {
    return new Translation3d(position[2], -position[0], position[1]);
  }

  // Gets the Rotation of the Quest.
  public Rotation3d getQuestNavRotation(float[] angles) {
    return new Rotation3d(
        Units.degreesToRadians(-angles[2]),
        Units.degreesToRadians(angles[0]),
        Units.degreesToRadians(-angles[1]));
  }

  private Pose3d getQuestNavPose(float[] position, float[] angles) {
    var oculousPositionCompensated = getQuestNavTranslation(position); // 6.5
    return new Pose3d(oculousPositionCompensated, getQuestNavRotation(angles));
  }

  /** Process heartbeat requests from Quest and respond with the same ID */
  public void processHeartbeat() {
    double requestId = heartbeatRequestSub.get();
    // Only respond to new requests to avoid flooding
    if (requestId > 0 && requestId != lastProcessedHeartbeatId) {
      heartbeatResponsePub.set(requestId);
      lastProcessedHeartbeatId = requestId;
    }
  }

  public void resetPose(Pose3d pose) {
    questMosi.accept(3);

    questNavRawToFieldCoordinateSystem =
        pose.getTranslation()
            .minus(lastPose3d.getTranslation().minus(questNavRawToFieldCoordinateSystem));

    count = 0;
    idx = 0;
  }

  public void resetHeading(Rotation2d heading) {
    questMosi.accept(3);

    gyroResetAngle =
        (lastPose3d.getRotation().minus(gyroResetAngle).minus(new Rotation3d(heading)))
            .unaryMinus();
  }

  public void resetHeading() {
    resetHeading(
        DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.kPi : Rotation2d.kZero);
  }

  public void resetBlue() {
    resetHeading(Rotation2d.kZero);
  }
}
