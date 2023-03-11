// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.lights;

import org.first5924.frc2023.subsystems.lights.LightsSubsystem;

import com.ctre.phoenix.led.Animation;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetLightsColorAndAnimation extends InstantCommand {
  private final LightsSubsystem mLights;
  private final int mR;
  private final int mG;
  private final int mB;
  private final Animation mAnimation;

  public SetLightsColorAndAnimation(LightsSubsystem lights, int r, int g, int b, Animation animation) {
    mLights = lights;
    mR = r;
    mG = g;
    mB = b;
    mAnimation = animation;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLights);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mLights.setColor(mR, mG, mB);
    if (mAnimation == null) {
      mLights.clearAnimation();
    } else {
      mLights.animate(mAnimation);
    }
  }
}
