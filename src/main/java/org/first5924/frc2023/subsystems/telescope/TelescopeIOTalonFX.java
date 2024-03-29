// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.telescope;

import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.constants.TelescopeConstants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class TelescopeIOTalonFX implements TelescopeIO {
    private final WPI_TalonFX mTelescopeTalon = new WPI_TalonFX(TelescopeConstants.kTalonPort);

    public TelescopeIOTalonFX() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = 42;
        config.supplyCurrLimit.triggerThresholdCurrent = 42;
        config.supplyCurrLimit.triggerThresholdTime = 0.1;

        mTelescopeTalon.configAllSettings(config);
        mTelescopeTalon.enableVoltageCompensation(false);
        mTelescopeTalon.setNeutralMode(NeutralMode.Brake);
        mTelescopeTalon.setInverted(true);
        mTelescopeTalon.set(0);
    }

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        inputs.telescopeExtensionInches = mTelescopeTalon.getSelectedSensorPosition() / RobotConstants.kTalonFXIntegratedSensorCPR / TelescopeConstants.kGearRatio * TelescopeConstants.kSprocketCircumferenceInches;
        inputs.telescopeExtensionInchesPerSecond = mTelescopeTalon.getSelectedSensorVelocity() / RobotConstants.kTalonFXIntegratedSensorCPR * 10 / TelescopeConstants.kGearRatio * TelescopeConstants.kSprocketCircumferenceInches;
        inputs.outputCurrent = mTelescopeTalon.getStatorCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        mTelescopeTalon.setVoltage(volts);
    }

    @Override
    public void setEncoderPosition(double position) {
        mTelescopeTalon.setSelectedSensorPosition(position);
    }
}