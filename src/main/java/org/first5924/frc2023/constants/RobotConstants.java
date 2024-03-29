// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.constants;

/** Add your docs here. */
public class RobotConstants {
    private RobotConstants() {}

    public static final int kNominalVoltage = 10;

    public static final int kThroughBoreCPR = 8192;
    public static final int kTalonFXIntegratedSensorCPR = 2048;

    public static final Mode kCurrentMode = Mode.REAL;

    public static enum Mode {
        /** Running on a real robot. */
        REAL,

        /** Running a physics simulator. */
        SIM,

        /** Replaying from a log file. */
        REPLAY
    }
}
