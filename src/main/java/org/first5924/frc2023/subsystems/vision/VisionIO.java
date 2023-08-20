// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Transform3d;

/** Add your docs here. */
public interface VisionIO {
    @AutoLog
    public static class VisionIOInputs {
        public boolean hasTarget = false;
        public double yAngle = 0;
        public double bestTargetID = 0;
        public double numberOfTargets = 0;
        public double xAngle = 0;
        public double tilt = 0;
        public double xDistance = 0;
        public double yDistance = 0;
        public double zDistance = 0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(VisionIOInputs inputs) {
    }


}
