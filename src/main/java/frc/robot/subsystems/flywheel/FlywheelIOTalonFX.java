package frc.robot.subsystems.flywheel;

import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelGains;
import frc.robot.subsystems.flywheel.FlywheelConstants.FlywheelHardwareConfig;
import java.util.ArrayList;

public class FlywheelIOTalonFX implements FlywheelIO {
  private final String name;

  private final TalonFX[] motors = new TalonFX[] {};

  private TalonFXConfiguration leaderConfig;

  private double velocitySetpoint = 0.0;

  private VoltageOut voltageRequest;
  private VelocityVoltage velocityRequest;

  private final StatusSignal<AngularVelocity> velocity;

  private final ArrayList<StatusSignal<Angle>> positions = new ArrayList<>();
  private final ArrayList<StatusSignal<AngularVelocity>> velocities = new ArrayList<>();

  private final ArrayList<StatusSignal<Voltage>> voltages = new ArrayList<>();
  private final ArrayList<StatusSignal<Current>> currents = new ArrayList<>();

  private boolean[] motorsConnected;

  private double[] motorPositions;
  private double[] motorVelocities;

  private double[] motorVoltages;
  private double[] motorCurrents;

  private Alert[] motorAlerts;

  public FlywheelIOTalonFX(String name, FlywheelHardwareConfig config) {
    this.name = name;

    motors[0] = new TalonFX(config.canIds()[0], config.canBus());
    leaderConfig =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(
                        config.reversed()[0]
                            ? InvertedValue.CounterClockwise_Positive
                            : InvertedValue.CounterClockwise_Positive))
            .withFeedback(
                new FeedbackConfigs()
                    .withRotorToSensorRatio(config.gearRatio())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder));

    tryUntilOk(5, () -> motors[0].getConfigurator().apply(leaderConfig));

    velocity = motors[0].getVelocity();
    positions.add(motors[0].getPosition());
    velocities.add(motors[0].getVelocity());

    voltages.add(motors[0].getSupplyVoltage());
    currents.add(motors[0].getStatorCurrent());

    motorAlerts[0] =
        new Alert(
            name + " Leader Motor Disconnected! CAN ID: " + config.canIds()[0], AlertType.kError);

    for (int i = 1; i < config.canIds().length; i++) {
      motors[i] = new TalonFX(config.canIds()[i], config.canBus());
      motors[i].setControl(new Follower(i, config.reversed()[i]));

      motorAlerts[i] =
          new Alert(
              name + " Follower Motor " + i + " Disconnected! CAN ID: " + config.canIds()[i],
              AlertType.kError);

      positions.add(motors[i].getPosition());
      velocities.add(motors[i].getVelocity());

      voltages.add(motors[i].getSupplyVoltage());
      currents.add(motors[i].getStatorCurrent());
    }
  }

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.velocity = motors[0].getVelocity().getValueAsDouble();

    inputs.velocitySetpoint = velocitySetpoint;

    for (int i = 0; i < motors.length; i++) {
      motorsConnected[i] =
          BaseStatusSignal.refreshAll(
                  velocity, positions.get(i), velocities.get(i), voltages.get(i), currents.get(i))
              .isOK();

      motorPositions[i] = positions.get(i).getValueAsDouble();
      motorVelocities[i] = velocities.get(i).getValueAsDouble();

      motorVoltages[i] = voltages.get(i).getValueAsDouble();
      motorCurrents[i] = motors[i].getStatorCurrent().getValueAsDouble();

      motorAlerts[i].set(motorsConnected[i]);
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

    motors[0].setControl(velocityRequest.withVelocity(velocity));
  }

  @Override
  public void setVoltage(double voltage) {
    motors[0].setControl(voltageRequest.withOutput(voltage));
  }

  @Override
  public void setGains(FlywheelGains gains) {
    motors[0]
        .getConfigurator()
        .apply(
            new Slot0Configs()
                .withKP(gains.kP())
                .withKI(gains.kI())
                .withKD(gains.kD())
                .withKV(gains.kV())
                .withKA(gains.kA())
                .withKS(gains.kS()));
  }

  @Override
  public String getName() {
    return name;
  }
}