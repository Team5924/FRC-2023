// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.util;

import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class BeamBreak {
    private final DigitalInput mBeamBreak;

    public BeamBreak(int channel) {
        mBeamBreak = new DigitalInput(channel);
    }

    public boolean isBeamBroken() {
        return mBeamBreak.get();
    }
}
