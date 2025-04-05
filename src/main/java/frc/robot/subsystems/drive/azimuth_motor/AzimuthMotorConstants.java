package frc.robot.subsystems.drive.azimuth_motor;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.position_joint.PositionJointConstants.EncoderType;

public class AzimuthMotorConstants {
  public static final String canBusName = "";

  public record AzimuthMotorGains(
      double kP,
      double kI,
      double kD,
      double kS,
      double kV,
      double kA,
      double kMaxVelo,
      double kMaxAccel) {}

  public record AzimuthMotorHardwareConfig(
      int[] canIds,
      boolean[] reversed,
      double gearRatio,
      double currentLimit,
      EncoderType encoderType,
      int encoderID,
      Rotation2d encoderOffset,
      String canBus) {}

  public static final AzimuthMotorHardwareConfig FRONT_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {4},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_ANALOG,
          2,
          Rotation2d.fromRotations(-0.410),
          canBusName);

  public static final AzimuthMotorHardwareConfig FRONT_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {2},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_ANALOG,
          0,
          Rotation2d.fromRotations(0.257),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_LEFT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {6},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_ANALOG,
          1,
          Rotation2d.fromRotations(-0.141),
          canBusName);

  public static final AzimuthMotorHardwareConfig BACK_RIGHT_CONFIG =
      new AzimuthMotorHardwareConfig(
          new int[] {8},
          new boolean[] {true},
          DriveConstants.steerMotorGearRatio,
          40,
          EncoderType.EXTERNAL_ANALOG,
          3,
          Rotation2d.fromRotations(-0.168),
          canBusName);

  public static final AzimuthMotorGains FRONT_LEFT_GAINS =
      new AzimuthMotorGains(10, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains FRONT_RIGHT_GAINS =
      new AzimuthMotorGains(10, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains BACK_LEFT_GAINS =
      new AzimuthMotorGains(10, 0, 0, 0, 2.5, 0, 5, 5);

  public static final AzimuthMotorGains BACK_RIGHT_GAINS =
      new AzimuthMotorGains(10, 0, 0, 0, 2.5, 0, 5, 5);
}
