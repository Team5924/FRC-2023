// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.subsystems.lights;

import org.first5924.frc2023.constants.LightsConstants;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

/** Add your docs here. */
public class LightsIOReal implements LightsIO {
    private final CANdle mCANdle = new CANdle(LightsConstants.kCANdlePort);
    public LightsIOReal() {

    }

    @Override
    public void setColor(int r, int g, int b) {
        mCANdle.setLEDs(r, g, b);
    }

    @Override
    public void animate(Animation animation) {
        mCANdle.animate(animation);
    }
}
