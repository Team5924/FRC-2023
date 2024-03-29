// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.grabber;

import org.littletonrobotics.junction.AutoLog;

public interface GrabberIO {
    @AutoLog
    public static class GrabberIOInputs {
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(GrabberIOInputs inputs) {
    }

    public default void setPercent(double percent) {
    }
}