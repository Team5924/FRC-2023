package org.first5924.frc2023.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface DriveIO {
    @AutoLog
    public static class DriveIOInputs {
        public double leftPositionMeters = 0.0;
        public double rightPositionMeters = 0.0;

        public double leftVelocityMetersPerSec = 0.0;
        public double rightVelocityMetersPerSec = 0.0;

        public double pigeonRotationDegrees = 0.0;

        public double frontEstimatedRobotPoseTranslationX = 0.0;
        public double frontEstimatedRobotPoseTranslationY = 0.0;
        public double frontEstimatedRobotPoseRotationX = 0.0;
        public double frontEstimatedRobotPoseRotationY = 0.0;
        public double frontEstimatedRobotPoseTimestampSeconds = 0.0;
        public double[] frontCameraToTargetsTranslationX;
        public double[] frontCameraToTargetsTranslationY;
        public double[] frontCameraToTargetsTranslationZ;

        public double backEstimatedRobotPoseTranslationX = 0.0;
        public double backEstimatedRobotPoseTranslationY = 0.0;
        public double backEstimatedRobotPoseRotationX = 0.0;
        public double backEstimatedRobotPoseRotationY = 0.0;
        public double backEstimatedRobotPoseTimestampSeconds = 0.0;
        public double[] backCameraToTargetsTranslationX;
        public double[] backCameraToTargetsTranslationY;
        public double[] backCameraToTargetsTranslationZ;
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
