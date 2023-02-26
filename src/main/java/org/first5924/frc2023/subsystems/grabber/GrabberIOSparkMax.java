// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.grabber;

import org.first5924.frc2023.constants.GrabberConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class GrabberIOSparkMax implements GrabberIO {
    private final CANSparkMax mGrabberSpark = SparkMaxFactory.createSparkMax(GrabberConstants.kSparkPort, MotorType.kBrushed, IdleMode.kBrake, 42);

    public GrabberIOSparkMax() {

    }

    @Override
    public void setPercent(double percent) {
        mGrabberSpark.set(percent);
    }
}
