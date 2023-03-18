// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.lib.util;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class JoystickToOutput {
    private JoystickToOutput() {};

    public static double calculateLinear(double joystick, double deadband) {
        return MathUtil.applyDeadband(joystick, deadband);
    }

    public static double calculateSquared(double joystick, double deadband) {
        return MathUtil.applyDeadband(Math.signum(joystick) * Math.pow(joystick, 2), deadband);
    }
}
