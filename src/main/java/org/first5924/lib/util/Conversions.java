// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.util;

/** Add your docs here. */
public class Conversions {
    private Conversions() {};

    // Length to rotation / rotation to length conversions

    public static double metersToDegrees(double robotMeters, double wheelCircumference) {
        double rotations = robotMeters / wheelCircumference;
        double degrees = rotations * 360;
        return degrees;
    }

    public static double degreesToMeters(double degrees, double wheelCircumference) {
        double rotations = degrees / 360;
        double meters = rotations * wheelCircumference;
        return meters;
    }

    // Length over time to rotation over time / rotation over time to length over time conversions

    public static double MPSToRotationsPerSecond(double MPS, double wheelCircumference) {
        double rotationsPerSecond = MPS / wheelCircumference;
        return rotationsPerSecond;
    }

    public static double rotationsPerSecondToMPS(double rotationsPerSecond, double wheelCircumference) {
        double MPS = rotationsPerSecond * wheelCircumference;
        return MPS;
    }

    public static double MPSToDegreesPerSecond(double MPS, double wheelCircumference) {
        double rotationsPerSecond = MPSToRotationsPerSecond(MPS, wheelCircumference);
        double degreesPerSecond = rotationsPerSecond * 360;
        return degreesPerSecond;
    }

    public static double degreesPerSecondToMPS(double degreesPerSecond, double wheelCircumference) {
        double rotationsPerSecond = degreesPerSecond / 360;
        double MPS = rotationsPerSecondToMPS(rotationsPerSecond, wheelCircumference);
        return MPS;
    }
}
