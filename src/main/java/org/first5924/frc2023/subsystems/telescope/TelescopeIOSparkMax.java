// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.telescope;

import org.first5924.frc2023.constants.TelescopeConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class TelescopeIOSparkMax implements TelescopeIO {
    private final CANSparkMax mTelescopeSpark = SparkMaxFactory.createSparkMax(TelescopeConstants.kSparkMaxPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final RelativeEncoder mEncoder = mTelescopeSpark.getEncoder();

    public TelescopeIOSparkMax() {}

    @Override
    public void updateInputs(TelescopeIOInputs inputs) {
        //inputs.telescopePositionDegrees = mEncoder.getPosition() * 360 / TelescopeConstants.kGearRatio;
    }

    @Override
    public void setPercent(double percent) {
        mTelescopeSpark.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        mTelescopeSpark.setVoltage(volts);
    }

    @Override
    public void setEncoderPosition(double position) {
        mEncoder.setPosition(position);
    }
}