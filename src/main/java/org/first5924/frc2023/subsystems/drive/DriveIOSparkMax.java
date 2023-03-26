// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Encoder;

/** Add your docs here. */
public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax mLeftFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mLeftBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);

    private final Encoder mLeftEncoder = new Encoder(DriveConstants.kLeftThroughBoreA, DriveConstants.kLeftThroughBoreB);
    private final Encoder mRightEncoder = new Encoder(DriveConstants.kRightThroughBoreA, DriveConstants.kRightThroughBoreB, true);

    private final WPI_Pigeon2 mPigeon2 = new WPI_Pigeon2(DriveConstants.kPigeon2Port);

    public DriveIOSparkMax() {
        mLeftBackSpark.follow(mLeftFrontSpark);
        mRightBackSpark.follow(mRightFrontSpark);

        mLeftFrontSpark.setInverted(true);

        mLeftEncoder.setReverseDirection(true);
        mRightEncoder.setReverseDirection(false);

        mPigeon2.configMountPose(-90, 0, 0);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionMeters = mLeftEncoder.getDistance() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.rightPositionMeters = mRightEncoder.getDistance() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.leftVelocityMetersPerSec = mLeftEncoder.getRate() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.rightVelocityMetersPerSec = mRightEncoder.getRate() / 2048 * DriveConstants.kWheelCircumferenceMeters;
        inputs.pigeonYaw = mPigeon2.getYaw();
        inputs.pigeonPitch = mPigeon2.getPitch();
    }

    @Override
    public void setPigeonYaw(double yaw) {
        mPigeon2.setYaw(yaw);
    }

    @Override
    public void resetEncoders() {
        mLeftEncoder.reset();
        mRightEncoder.reset();
    }

    @Override
    public void setVoltage(double leftVolts, double rightVolts) {
        mLeftFrontSpark.setVoltage(leftVolts);
        mRightFrontSpark.setVoltage(rightVolts);
    }

    @Override
    public void setPercent(double leftPercent, double rightPercent) {
        mLeftFrontSpark.set(leftPercent);
        mRightFrontSpark.set(rightPercent);
    }
}
