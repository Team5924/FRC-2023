// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
    private UsbCamera frontCamera = CameraServer.startAutomaticCapture("Front Camera", 0);
    // private UsbCamera backCamera = CameraServer.startAutomaticCapture("Back Camera", 1);

    public VisionIOReal() {
        frontCamera.setFPS(30);
        frontCamera.setBrightness(70);
        frontCamera.setResolution(320, 240);

        // backCamera.setFPS(20);
        // backCamera.setBrightness(70);
        // backCamera.setResolution(256, 192);
    }
}
