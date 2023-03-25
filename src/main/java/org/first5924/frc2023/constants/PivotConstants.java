// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

/** Add your docs here. */
public class PivotConstants {
    private PivotConstants() {}

    public static final int kLeaderSparkPort = 8;
    public static final int kFollowerSparkPort = 7;

    public static final double kGearRatio = 131.22;

    public static final double kSpeedMultiplier = 0.16; //lower this, delete the unused constants after

    public static final double kStartingDegrees = 0;

    //All specific pivot angles
    public static final double kSingleSubstation = 60;
    public static final double kDoubleSubstation = 57;

    public static final double kHolding = 40;

    public static final double kGroundPickupCube = 100;
    public static final double kGroundPickupCone = 100;

    public static final double kMiddleGridCube =  73;
    public static final double kMiddleGridCone = 61;
    public static final double kTopGridCube = 61;
}
