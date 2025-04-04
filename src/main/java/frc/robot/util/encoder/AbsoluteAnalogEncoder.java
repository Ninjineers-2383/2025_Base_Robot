package frc.robot.util.encoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.AnalogEncoder;

public class AbsoluteAnalogEncoder implements IAbsoluteEncoder {
  private final AnalogEncoder encoder;

  public AbsoluteAnalogEncoder(int port) {
    this.encoder = new AnalogEncoder(port);
  }

  @Override
  public Rotation2d getAbsoluteAngle() {
    return Rotation2d.fromRotations(this.encoder.get());
  }

  @Override
  public boolean isConnected() {
    return true;
  }
}
