// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.drive;

import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.lib.util.SparkMaxFactory;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAlternateEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** Add your docs here. */
public class DriveIOSparkMax implements DriveIO {
    private final CANSparkMax mLeftFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightFrontSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightFrontSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mLeftBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kLeftBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final CANSparkMax mRightBackSpark = SparkMaxFactory.createSparkMax(DriveConstants.kRightBackSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);

    private final RelativeEncoder mLeftThroughBore = mLeftBackSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, RobotConstants.kThroughBoreCPR);
    private final RelativeEncoder mRightThroughBore = mRightBackSpark.getAlternateEncoder(SparkMaxAlternateEncoder.Type.kQuadrature, RobotConstants.kThroughBoreCPR);

    private final WPI_Pigeon2 mPigeon2 = new WPI_Pigeon2(DriveConstants.kPigeon2Port);

    public DriveIOSparkMax() {
        mLeftBackSpark.follow(mLeftFrontSpark);
        mRightBackSpark.follow(mRightFrontSpark);

        mLeftFrontSpark.setInverted(true);

        mLeftThroughBore.setInverted(true);
    }

    @Override
    public void updateInputs(DriveIOInputs inputs) {
        inputs.leftPositionMeters = mLeftThroughBore.getPosition() * DriveConstants.kWheelCircumferenceMeters;
        inputs.rightPositionMeters = mRightThroughBore.getPosition() * DriveConstants.kWheelCircumferenceMeters;
        inputs.leftVelocityMetersPerSec = mLeftThroughBore.getVelocity() * DriveConstants.kWheelCircumferenceMeters / 60;
        inputs.rightVelocityMetersPerSec = mRightThroughBore.getVelocity() * DriveConstants.kWheelCircumferenceMeters / 60;
        inputs.pigeonRotationDeg = mPigeon2.getRotation2d().getDegrees();
    }

    @Override
    public void setPigeonYaw(double yaw) {
        mPigeon2.setYaw(yaw);
    }

    @Override
    public void setThroughBoreRotations(double leftRotation, double rightRotation) {
        mLeftThroughBore.setPosition(leftRotation);
        mRightThroughBore.setPosition(rightRotation);
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
