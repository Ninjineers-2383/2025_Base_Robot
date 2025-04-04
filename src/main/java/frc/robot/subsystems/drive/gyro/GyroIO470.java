package frc.robot.subsystems.drive.gyro;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import frc.robot.subsystems.drive.odometry_threads.PhoenixOdometryThread;
import java.util.Queue;

public class GyroIO470 implements GyroIO {
  private final ADIS16470_IMU gyro;
  private final Queue<Double> yawPositionQueue;
  private final Queue<Double> yawTimestampQueue;

  public GyroIO470() {
    gyro = new ADIS16470_IMU();

    yawTimestampQueue = PhoenixOdometryThread.getInstance().makeTimestampQueue();
    yawPositionQueue =
        PhoenixOdometryThread.getInstance().registerSignal(() -> gyro.getAngle(IMUAxis.kYaw));
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = true;
    inputs.yawPosition = Rotation2d.fromDegrees(-gyro.getAngle(IMUAxis.kYaw));
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(-gyro.getRate(IMUAxis.kYaw));

    inputs.odometryYawTimestamps =
        yawTimestampQueue.stream().mapToDouble((Double value) -> value).toArray();
    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(-value))
            .toArray(Rotation2d[]::new);
    yawTimestampQueue.clear();
    yawPositionQueue.clear();
  }
}
