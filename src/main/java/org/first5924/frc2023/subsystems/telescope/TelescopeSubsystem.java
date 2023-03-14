package org.first5924.frc2023.subsystems.telescope;

import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TelescopeSubsystem extends SubsystemBase {
  private final TelescopeIO io;
  private final TelescopeIOInputsAutoLogged inputs = new TelescopeIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(0.001, 0, 0);

  /** Creates a new TelescopeSubsystem. */
  public TelescopeSubsystem(TelescopeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    SmartDashboard.putBoolean("Telescope In Forward Slow Zone?", isInForwardSlowZone());
    SmartDashboard.putBoolean("Telescope In Backward Slow Zone?", isInBackwardSlowZone());
    Logger.getInstance().processInputs("Telescope", inputs);
  }

  public double getTelescopeExtensionInches() {
   return inputs.telescopeExtensionInches;
  }

  public double getTelescopeExtensionInchesPerSecond() {
   return inputs.telescopeExtensionInchesPerSecond;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
    SmartDashboard.putNumber("Telescope percent", percent);
  }

  public void setPosition(double position) {
    io.setVoltage(MathUtil.clamp(mPID.calculate(getTelescopeExtensionInches(), position), -3, 3));
  }

  public void setEncoderFromTelescopeExtensionInches(double extensionInches) {
    io.setEncoderPosition(extensionInches * TelescopeConstants.kGearRatio / TelescopeConstants.kSprocketCircumferenceInches * RobotConstants.kTalonFXIntegratedSensorCPR);
  }

  public boolean isInForwardSlowZone() {
    return getTelescopeExtensionInches() >= TelescopeConstants.kMaxForwardExtensionInches - TelescopeConstants.kSlowZoneInches;
  }

  public boolean isInBackwardSlowZone() {
    return getTelescopeExtensionInches() <= TelescopeConstants.kMaxBackwardExtensionInches + TelescopeConstants.kSlowZoneInches;
  }
}
