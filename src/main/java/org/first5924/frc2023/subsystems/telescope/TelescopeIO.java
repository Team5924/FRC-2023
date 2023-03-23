// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.telescope;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TelescopeIO {
    @AutoLog
    public static class TelescopeIOInputs {
        public double telescopeExtensionInches = 0.0;
        public double telescopeExtensionInchesPerSecond = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(TelescopeIOInputs inputs) {
    }

    public default void setVoltage(double volts) {
    }

    public default void setEncoderPosition(double position) {
    }
}
