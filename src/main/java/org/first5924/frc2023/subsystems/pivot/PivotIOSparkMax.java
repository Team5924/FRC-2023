// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.pivot;


import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class PivotIOSparkMax implements PivotIO {
    private final CANSparkMax mLeaderSpark = SparkMaxFactory.createSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mFollowerSpark = SparkMaxFactory.createSparkMax(PivotConstants.kFollowerSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final RelativeEncoder mEncoder = mLeaderSpark.getEncoder();

    public PivotIOSparkMax() {
        mFollowerSpark.follow(mLeaderSpark);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotPositionDegrees = mEncoder.getPosition() * 360 / PivotConstants.kGearRatio;
    }

    @Override
    public void setPercent(double percent) {
        mLeaderSpark.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        mLeaderSpark.setVoltage(volts);
    }

    @Override
    public void setEncoderPosition(double position) {
        mEncoder.setPosition(position);
    }
}
