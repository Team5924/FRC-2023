// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.pivot;


import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.RobotConstants;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/** Add your docs here. */
public class PivotIOSparkMax implements PivotIO {
    private final WPI_TalonFX mLeaderTalon = new WPI_TalonFX(PivotConstants.kLeaderTalonPort);
    private final WPI_TalonFX mFollowerTalon = new WPI_TalonFX(PivotConstants.kFollowerTalonPort);

    public PivotIOSparkMax() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.voltageCompSaturation = RobotConstants.kNominalVoltage;
        config.supplyCurrLimit.enable = true;
        config.supplyCurrLimit.currentLimit = 42;
        config.supplyCurrLimit.triggerThresholdCurrent = 42;
        config.supplyCurrLimit.triggerThresholdTime = 0.1;

        mLeaderTalon.configAllSettings(config);
        mLeaderTalon.enableVoltageCompensation(true);
        mLeaderTalon.set(0);

        mFollowerTalon.follow(mLeaderTalon);
        mFollowerTalon.setInverted(TalonFXInvertType.FollowMaster);
    }

    @Override
    public void updateInputs(PivotIOInputs inputs) {
        inputs.pivotPositionDegrees = mLeaderTalon.getSelectedSensorPosition() / RobotConstants.kTalonFXIntegratedSensorCPR * 360 / PivotConstants.kGearRatio;
        inputs.pivotVelocityDegreesPerSecond = mLeaderTalon.getSelectedSensorVelocity() / RobotConstants.kTalonFXIntegratedSensorCPR * 360 * 10 / PivotConstants.kGearRatio;
    }

    @Override
    public void setPercent(double percent) {
        mLeaderTalon.set(percent);
    }

    @Override
    public void setVoltage(double volts) {
        mLeaderTalon.setVoltage(volts);
    }

    @Override
    public void setEncoderPosition(double position) {
        mLeaderTalon.setSelectedSensorPosition(position);
    }
}
