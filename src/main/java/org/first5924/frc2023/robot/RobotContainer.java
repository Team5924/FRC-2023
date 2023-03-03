// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.robot;

import org.first5924.frc2023.commands.autonomous.AutoRoutines;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceAroundClimbAuto;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceOverClimbAuto;
import org.first5924.frc2023.commands.autonomous.routines.StationaryAuto;
import org.first5924.frc2023.commands.autonomous.routines.TwoPieceClimbAuto;
import org.first5924.frc2023.commands.drive.CurvatureDrive;
import org.first5924.frc2023.commands.drive.TurnInPlace;
import org.first5924.frc2023.commands.pivot.RotatePivot;
import org.first5924.frc2023.commands.grabber.Grab;
import org.first5924.frc2023.commands.grabber.Release;
import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.subsystems.drive.DriveIO;
import org.first5924.frc2023.subsystems.drive.DriveIOSparkMax;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.grabber.GrabberIO;
import org.first5924.frc2023.subsystems.grabber.GrabberIOSparkMax;
import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;
import org.first5924.frc2023.subsystems.pivot.PivotIO;
import org.first5924.frc2023.subsystems.pivot.PivotIOSparkMax;
import org.first5924.frc2023.subsystems.pivot.PivotSubsystem;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // * SUBSYSTEMS
  private final DriveSubsystem mDrive;
  private final PivotSubsystem mPivot;
  private final GrabberSubsystem mGrabber;

  private final LoggedDashboardChooser<Alliance> mAllianceChooser = new LoggedDashboardChooser<>("AllianceChooser");
  private final LoggedDashboardChooser<AutoRoutines> mAutoChooser = new LoggedDashboardChooser<>("AutoChooser");

  // * CONTROLLER & BUTTONS
  private final CommandXboxController mDriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController mOperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.kCurrentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        mDrive = new DriveSubsystem(new DriveIOSparkMax());
        mPivot = new PivotSubsystem(new PivotIOSparkMax());
        mGrabber = new GrabberSubsystem(new GrabberIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        mDrive = new DriveSubsystem(new DriveIO() {});
        mPivot = new PivotSubsystem(new PivotIO() {});
        mGrabber = new GrabberSubsystem(new GrabberIO() {});
        break;

      // Replayed robot, disable IO implementations
      default:
        mDrive = new DriveSubsystem(new DriveIO() {});
        mPivot = new PivotSubsystem(new PivotIO() {});
        mGrabber = new GrabberSubsystem(new GrabberIO() {});
        break;
    }

    mAllianceChooser.addDefaultOption("Blue", Alliance.Blue);
    mAllianceChooser.addOption("Red", Alliance.Red);

    mAutoChooser.addDefaultOption("One Piece Around Climb", AutoRoutines.onePieceAroundClimb);
    mAutoChooser.addOption("One Piece Over Climb", AutoRoutines.onePieceOverClimb);
    mAutoChooser.addOption("Two Piece Climb", AutoRoutines.twoPieceClimb);
    mAutoChooser.addOption("Stationary", AutoRoutines.stationary);

    //mPivot.setDefaultCommand(new RotatePivot(mPivot, mOperatorController::getRightY));

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    mDriverController.leftBumper().whileTrue(new TurnInPlace(mDrive, mDriverController::getLeftY, mDriverController::getRightX));

    mPivot.setDefaultCommand(new RotatePivot(mPivot, mOperatorController::getLeftY));
    mOperatorController.leftTrigger().whileTrue(new Release(mGrabber));
    mOperatorController.rightTrigger().whileTrue(new Grab(mGrabber));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch (mAutoChooser.get()) {
      case onePieceAroundClimb:
        return new OnePieceAroundClimbAuto(mDrive, mAllianceChooser.get());
      case onePieceOverClimb:
        return new OnePieceOverClimbAuto(mDrive, mAllianceChooser.get());
      case twoPieceClimb:
        return new TwoPieceClimbAuto(mDrive, mAllianceChooser.get());
      case stationary:
        return new StationaryAuto();
      default:
      return new OnePieceAroundClimbAuto(mDrive, mAllianceChooser.get());
    }
  }
}
