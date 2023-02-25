package org.first5924.frc2023.subsystems.telescope;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
  private final TelescopeIO io;
  private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();

  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem(TelescopeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.getInstance().processInputs("Telescope", inputs);
  }

  public void runTelescope(double percent) {
    io.setPercent(percent);
  }
}
