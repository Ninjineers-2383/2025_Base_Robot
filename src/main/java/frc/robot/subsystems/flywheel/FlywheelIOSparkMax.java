package frc.robot.subsystems.flywheel;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.EncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;
import frc.robot.util.feedforwards.TunableSimpleMotorFeedforward;

public class FlywheelIOSparkMax implements FlywheelIO {
  private final String name;

  private final SparkMax[] motors;
  private final SparkBaseConfig leaderConfig;

  private final boolean[] motorsConnected;

  private final double[] motorPositions;
  private final double[] motorVelocities;

  private final double[] motorVoltages;
  private final double[] motorCurrents;

  private final Alert[] motorAlerts;

  private TunableSimpleMotorFeedforward feedforward;

  private double velocitySetpoint = 0.0;

  public FlywheelIOSparkMax(String name, FlywheelHardwareConfig config) {
    this(name, config, true);
  }

  public FlywheelIOSparkMax(String name, FlywheelHardwareConfig config, boolean isBrushless) {
    this.name = name;

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
              .inverted(config.reversed()[0]);

    } else {
      leaderConfig =
          new SparkMaxConfig()
              .apply(
                  new EncoderConfig()
                      .positionConversionFactor(1.0 / config.gearRatio())
                      .velocityConversionFactor(1.0 / (60.0 * config.gearRatio()))
                      .inverted(config.reversed()[0]));
    }

    motors[0].configure(
        leaderConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

    motorAlerts[0] =
        new Alert(
            name,
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0],
            AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] =
          new SparkMax(config.canIds()[i], isBrushless ? MotorType.kBrushless : MotorType.kBrushed);
      motors[i].configure(
          new SparkMaxConfig().follow(motors[0], config.reversed()[i]),
          ResetMode.kNoResetSafeParameters,
          PersistMode.kNoPersistParameters);

      motorAlerts[i] =
          new Alert(
              name,
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);
    }

    feedforward = new TunableSimpleMotorFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocity = motors[0].getEncoder().getVelocity();

    inputs.desiredVelocity = velocitySetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] = motors[i].getLastError() == REVLibError.kOk;

      motorPositions[i] = motors[i].getEncoder().getPosition();
      motorVelocities[i] = motors[i].getEncoder().getVelocity();

      motorVoltages[i] = motors[i].getAppliedOutput() * 12;
      motorCurrents[i] = motors[i].getOutputCurrent();

      motorAlerts[i].set(!motorsConnected[i]);
    }

    inputs.motorsConnected = motorsConnected;

    inputs.motorPositions = motorPositions;
    inputs.motorVelocities = motorVelocities;

    inputs.motorVoltages = motorVoltages;
    inputs.motorCurrents = motorCurrents;
  }

  @Override
  public void setVelocity(double velocity) {
    velocitySetpoint = velocity;

    motors[0]
        .getClosedLoopController()
        .setReference(
            velocitySetpoint,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            feedforward.calculateWithVelocities(motors[0].getEncoder().getVelocity(), velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setVoltage(voltage);
  }

  @Override
  public void setGains(FlywheelGains gains) {
    motors[0].configure(
        leaderConfig.apply(new ClosedLoopConfig().pid(gains.kP(), gains.kI(), gains.kD())),
        ResetMode.kNoResetSafeParameters,
        PersistMode.kNoPersistParameters);

    feedforward.setGains(gains.kS(), gains.kV(), gains.kA());

    System.out.println(name + " gains set to " + gains);
  }

  @Override
  public String getName() {
    return name;
  }
}
