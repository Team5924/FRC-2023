package org.first5924.frc2023.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionMeters = 0.0;
        public double rightPositionMeters = 0.0;
        public double leftVelocityMetersPerSec = 0.0;
        public double rightVelocityMetersPerSec = 0.0;
        public double pigeonRotationDeg = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(DriveIOInputs inputs) {
    }

    public default void setPigeonYaw(double yaw) {
    }

    public default void resetEncoders() {
    }

    public default void setVoltage(double leftVolts, double rightVolts) {
    }

    public default void setPercent(double leftPercent, double rightPercent) {
    }
}
