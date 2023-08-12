// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
    private DriveConstants() {}

    public static final int kLeftFrontSparkPort = 2;
    public static final int kLeftBackSparkPort = 3;
    public static final int kRightFrontSparkPort = 4;
    public static final int kRightBackSparkPort = 5;

    public static final int kPigeon2Port = 6;

    public static final int kLeftThroughBoreA = 2;
    public static final int kLeftThroughBoreB = 3;
    public static final int kRightThroughBoreA = 0;
    public static final int kRightThroughBoreB = 1;

    // Values generated from sysid
    public static final double ks = 0.12223;
    public static final double kv = 2.8236;
    public static final double ka = 0.10173;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTrackwidthMeters = Units.inchesToMeters(21.75);
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kWheelCircumferenceMeters = Units.inchesToMeters(5 * Math.PI);

    public static final double kTurnP = 0.005; 
    public static final double kPDriveVel = 1.1772;
    public static final double minTurn = .03;
}
