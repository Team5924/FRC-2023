// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.commands.autonomous.routines;

import org.first5924.frc2023.commands.drive.AutoEngageChargeStation;
import org.first5924.frc2023.constants.DriveConstants;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OnePieceClimbAuto extends SequentialCommandGroup {
  private final Trajectory mOnePieceA;

  /** Creates a new DriveOneMeter. */
  public OnePieceClimbAuto(DriveSubsystem drive, Alliance alliance) {
    mOnePieceA = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath("1 Piece Climb A", 2.5, 2), alliance);
    Logger.getInstance().recordOutput("One Piece Climb A", mOnePieceA);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> {
        drive.resetPosition(mOnePieceA.getInitialPose());
      }),
      new RamseteCommand(
        mOnePieceA,
        drive::getEstimatedRobotPose,
        new RamseteController(),
        new SimpleMotorFeedforward(DriveConstants.ks, DriveConstants.kv, DriveConstants.ka),
        DriveConstants.kKinematics,
        drive::getWheelSpeeds,
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        new PIDController(DriveConstants.kP, DriveConstants.kI, DriveConstants.kD),
        drive::setVoltage,
        drive
      ),
      new InstantCommand(() -> {
        drive.setVoltage(0, 0);
      }),
      new AutoEngageChargeStation(drive)
    );
  }
}
