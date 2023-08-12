// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

/** Add your docs here. */
public class VisionConstants {
    public VisionConstants() {}
    public static final double kCameraHeightMeters = 0.4191;
//top
    //public static final double kTargetHeightMeters = 1.5494;
//center
    public static final double kTargetHeightMeters = 0.381;
//bottom
    //public static final double kTargetHeightMeters = 0.381;
    
    public static final double kCameraPitchRadians = 0;

    public static final Transform3d robotToCam =
                new Transform3d(
                        new Translation3d(0.0889, 0.18415, 0.381),
                        new Rotation3d(
                                0, 0,
                                0)); // Cam mounted facing forward, half a meter forward of center, half a meter up
        // from center.
    public static final String cameraName = "PhotonUSBCamera";
    }

