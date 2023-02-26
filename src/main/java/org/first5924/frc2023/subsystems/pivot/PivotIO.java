// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;
/** Add your docs here. */
public interface PivotIO {
    @AutoLog
    public static class PivotIOInputs {
        public double motorPosition = 0.0;
    }

    /** Updates the set of loggable inputs */
    public default void updateInputs(PivotIOInputs inputs) {
    }

    public default void setPercent(double percent) {
    }
}
