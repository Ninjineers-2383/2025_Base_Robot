package frc.robot.subsystems.position_joint;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.subsystems.position_joint.PositionJointConstants.GravityType;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointGains;
import frc.robot.subsystems.position_joint.PositionJointConstants.PositionJointHardwareConfig;
import frc.robot.util.encoder.AbsoluteCancoder;
import frc.robot.util.encoder.AbsoluteMagEncoder;
import frc.robot.util.encoder.IAbsoluteEncoder;
import frc.robot.util.feedforwards.PositionJointFeedforward;
import frc.robot.util.feedforwards.TunableArmFeedforward;
import frc.robot.util.feedforwards.TunableElevatorFeedforward;
import java.util.function.DoubleSupplier;

public class PositionJointIOSparkMax implements PositionJointIO {
  private final String name;

  private final PositionJointHardwareConfig hardwareConfig;

  private final DoubleSupplier externalFeedforward;

  private final SparkMax[] motors;
  private final SparkBaseConfig leaderConfig;

  private final IAbsoluteEncoder externalEncoder;

  private final boolean[] motorsConnected;
  private boolean encoderConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;
  private final Alert encoderAlert;

  private final PositionJointFeedforward feedforward;
  private final double feedforward_position_addition;

  private double currentPosition = 0.0;
  private double positionSetpoint = 0.0;
  private double velocitySetpoint = 0.0;

  public PositionJointIOSparkMax(
      String name,
      PositionJointHardwareConfig config,
      DoubleSupplier externalFeedforward,
      boolean isBrushless) {
    this.name = name;
    hardwareConfig = config;
    this.externalFeedforward = externalFeedforward;

    assert config.canIds().length > 0 && (config.canIds().length == config.reversed().length);

    motors = new SparkMax[config.canIds().length];
    motorsConnected = new boolean[config.canIds().length];
    motorPositions = new double[config.canIds().length];
    motorVelocities = new double[config.canIds().length];
    motorVoltages = new double[config.canIds().length];
    motorCurrents = new double[config.canIds().length];
    motorAlerts = new Alert[config.canIds().length];

    motors[0] =
        new SparkMax(config.canIds()[0], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);

    if (isBrushless) {
      leaderConfig =
          new SparkMaxConfig()
              .apply(
                  new EncoderConfig()
                      .positionConversionFactor(1.0 / config.gearRatio())
                      .velocityConversionFactor(1.0 / (60.0 * config.gearRatio())))
              .inverted(config.reversed()[0])
              .idleMode(IdleMode.kBrake);

    } else {
      leaderConfig =
          new SparkMaxConfig()
              .apply(
                  new EncoderConfig()
                      .positionConversionFactor(1.0 / config.gearRatio())
                      .velocityConversionFactor(1.0 / (60.0 * config.gearRatio()))
                      .inverted(config.reversed()[0]))
              .idleMode(IdleMode.kBrake);
    }

    switch (config.encoderType()) {
      case INTERNAL:
        externalEncoder = new IAbsoluteEncoder() {};

        encoderAlert =
            new Alert(name, name + " does not use an external encoder 💀", AlertType.kInfo);

        motors[0].configure(
            leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        break;
      case EXTERNAL_CANCODER:
        externalEncoder =
            new AbsoluteCancoder(
                config.encoderID(),
                config.canBus(),
                new CANcoderConfiguration()
                    .withMagnetSensor(
                        new MagnetSensorConfigs()
                            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                            .withMagnetOffset(config.encoderOffset().getMeasure())));

        encoderAlert =
            new Alert(
                name,
                name + " CANCoder Disconnected! CAN ID: " + config.encoderID(),
                AlertType.kError);

        motors[0].configure(
            leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motors[0].getEncoder().setPosition(externalEncoder.getAbsoluteAngle().getRotations());
        break;
      case EXTERNAL_CANCODER_PRO:
        throw new IllegalArgumentException("EXTERNAL_CANCODER_PRO not supported on SparkMax");
      case EXTERNAL_DIO:
        externalEncoder = new AbsoluteMagEncoder(config.encoderID());

        encoderAlert =
            new Alert(
                name,
                name + " DIO Encoder Disconnected! DIO ID: " + config.encoderID(),
                AlertType.kWarning);

        motors[0].configure(
            leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motors[0]
            .getEncoder()
            .setPosition(
                externalEncoder.getAbsoluteAngle().plus(config.encoderOffset()).getRotations());
        break;
      case EXTERNAL_SPARK:
        externalEncoder = new IAbsoluteEncoder() {};

        encoderAlert =
            new Alert(name, name + " Internal SPARK Encoder Disconnected", AlertType.kWarning);

        leaderConfig.apply(
            new AbsoluteEncoderConfig()
                .positionConversionFactor(1.0)
                .velocityConversionFactor(1.0)
                .zeroOffset(config.encoderOffset().getRotations())
                .averageDepth(2));

        motors[0].configure(
            leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        motors[0]
            .getEncoder()
            .setPosition(
                motors[0].getAbsoluteEncoder().getPosition()
                    + config.encoderOffset().getRotations());
        break;

      default:
        externalEncoder = new IAbsoluteEncoder() {};
        encoderAlert =
            new Alert(name, name + " does not use an external encoder 💀", AlertType.kInfo);
        break;
    }

    motorAlerts[0] =
        new Alert(
            name,
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0],
            AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] =
          new SparkMax(config.canIds()[i], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
      motors[i].configure(
          new SparkMaxConfig().follow(motors[0], config.reversed()[i]).idleMode(IdleMode.kBrake),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

      motorAlerts[i] =
          new Alert(
              name,
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);
    }

    if (config.gravity() == GravityType.CONSTANT) {
      feedforward = new TunableElevatorFeedforward();
      feedforward_position_addition = 0.0;
    } else {
      feedforward = new TunableArmFeedforward();
      if (config.gravity() == GravityType.SINE) {
        feedforward_position_addition = -Math.PI / 2;
      } else {
        feedforward_position_addition = 0.0;
      }
    }
  }

  public PositionJointIOSparkMax(String name, PositionJointHardwareConfig config) {
    this(name, config, () -> 0, true);
  }

  @Override
  public void updateInputs(PositionJointIOInputs inputs) {
    currentPosition = motors[0].getEncoder().getPosition();

    inputs.outputPosition = currentPosition;
    inputs.rotorPosition = currentPosition * hardwareConfig.gearRatio();
    inputs.desiredPosition = positionSetpoint;

    inputs.velocity = motors[0].getEncoder().getVelocity();
    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;

      motorPositions[i] = motors[i].getEncoder().getPosition();
      motorVelocities[i] = motors[i].getEncoder().getVelocity();

      motorVoltages[i] = motors[i].getAppliedOutput() * RobotController.getBatteryVoltage();
      motorCurrents[i] = motors[i].getOutputCurrent();

      motorAlerts[i].set(!motorsConnected[i]);
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;

    switch (hardwareConfig.encoderType()) {
      case INTERNAL:
        encoderConnected = false;
        break;
      case EXTERNAL_CANCODER:
        encoderConnected = externalEncoder.isConnected();
        break;
      case EXTERNAL_CANCODER_PRO:
        encoderConnected = false;
        break;
      case EXTERNAL_DIO:
        encoderConnected = externalEncoder.isConnected();
        break;
      case EXTERNAL_SPARK:
        encoderConnected = motors[0].getLastError() == REVLibError.kOk;
        break;
    }

    encoderAlert.set(!encoderConnected);
    inputs.encoderConnected = encoderConnected;
  }

  @Override
  public void setPosition(double desiredPosition, double desiredVelocity) {
    positionSetpoint = desiredPosition;

    double ffposition = currentPosition + feedforward_position_addition;

    motors[0]
        .getClosedLoopController()
        .setReference(
            positionSetpoint,
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0,
            feedforward.calculate(ffposition, velocitySetpoint, desiredVelocity, 0.02)
                + externalFeedforward.getAsDouble());

    velocitySetpoint = desiredVelocity;
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(PositionJointGains gains) {
    feedforward.setGains(gains.kS(), gains.kG(), gains.kV(), gains.kA());

    motors[0].configure(
        leaderConfig.apply(new ClosedLoopConfig().pidf(gains.kP(), gains.kI(), gains.kD(), 0)),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public void resetPosition() {
    for (int i = 0; i < motors.length; i++) {
      motors[i].getEncoder().setPosition(0.0);
    }
  }

  @Override
  public String getName() {
    return name;
  }
}
