// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class VisionConstants {
    private VisionConstants() {}

    public static final Transform3d kRobotToFrontCam = new Transform3d(
        new Translation3d(-5.875, -6, 12.25),
        new Rotation3d());
    public static final Transform3d kRobotToBackCam = new Transform3d(
        new Translation3d(5.875, 6, 12.25),
        new Rotation3d(0, 0, Units.degreesToRadians(180))
    );
}
