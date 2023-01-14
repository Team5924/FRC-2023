// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/** Add your docs here. */
public class DriveConstants {
    public static final int kLeftFrontSparkPort = 1;
    public static final int kLeftBackSparkPort = 2;
    public static final int kRightFrontSparkPort = 3;
    public static final int kRightBackSparkPort = 4;

    public static final int kLeftCANCoderPort = 5;
    public static final int kRightCANCoderPort = 6;

    // Values generated from sysid
    public static final double ks = 0;
    public static final double kv = 0;
    public static final double ka = 0;

    public static final double kP = 0;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double kTrackwidthMeters = 0;
    public static final DifferentialDriveKinematics kKinematics = new DifferentialDriveKinematics(kTrackwidthMeters);

    public static final double kWheelCircumferenceMeters = 0;

    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
}
