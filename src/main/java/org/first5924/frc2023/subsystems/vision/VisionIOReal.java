// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.vision;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
    private UsbCamera camera = CameraServer.startAutomaticCapture("Camera", 0);

    public VisionIOReal() {
        camera.setFPS(30);
        camera.setBrightness(70);
        camera.setResolution(320, 240);
    }
}
