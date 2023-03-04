package org.first5924.frc2023.subsystems.telescope;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
  private final TelescopeIO io;
  private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(0.2, 0, 0);

  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem(TelescopeIO io) {
    this.io = io;
  }

  public double getEncoderPosition() {
   return inputs.encoderRotations;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public void setPosition(double position) {
    io.setVoltage(MathUtil.clamp(mPID.calculate(getEncoderPosition(), position), -3, 3));
  }

  public void setEncoderPosition(double position) {
    io.setEncoderPosition(position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Telescope", inputs);
  }
}
