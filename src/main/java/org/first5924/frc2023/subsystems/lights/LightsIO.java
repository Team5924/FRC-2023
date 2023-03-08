// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.lights;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix.led.Animation;

public interface LightsIO {
    @AutoLog
    public static class LightsIOInputs {
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(LightsIOInputs inputs) {
    }

    public default void setColor(int r, int g, int b) {
    }

    public default void animate(Animation animation) {
    }
}