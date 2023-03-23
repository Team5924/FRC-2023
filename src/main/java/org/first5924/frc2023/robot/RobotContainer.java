// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.robot;

import org.first5924.frc2023.commands.autonomous.AutoRoutines;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceAroundClimbAuto;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceMobilityAuto;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceOverClimbAuto;
import org.first5924.frc2023.commands.autonomous.routines.OnePieceStationaryAuto;
import org.first5924.frc2023.commands.autonomous.routines.NothingAuto;
import org.first5924.frc2023.commands.drive.CurvatureDrive;
import org.first5924.frc2023.commands.drive.TurnInPlace;
import org.first5924.frc2023.commands.telescope.ExtendAndRetractTelescope;
import org.first5924.frc2023.commands.telescope.SetTelescope;
import org.first5924.frc2023.commands.pivot.RotatePivot;
import org.first5924.frc2023.commands.pivot.SetPivot;
import org.first5924.frc2023.commands.grabber.Grab;
import org.first5924.frc2023.commands.grabber.Release;
import org.first5924.frc2023.commands.grabber.SlowRelease;
import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.constants.PivotConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.subsystems.drive.DriveIO;
import org.first5924.frc2023.subsystems.drive.DriveIOSparkMax;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeIO;
import org.first5924.frc2023.subsystems.telescope.TelescopeIOTalonFX;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;
import org.first5924.frc2023.subsystems.vision.VisionSubsystem;
import org.first5924.frc2023.subsystems.vision.VisionIO;
import org.first5924.frc2023.subsystems.vision.VisionIOReal;
import org.first5924.frc2023.subsystems.grabber.GrabberIO;
import org.first5924.frc2023.subsystems.grabber.GrabberIOSparkMax;
import org.first5924.frc2023.subsystems.grabber.GrabberSubsystem;
// import org.first5924.frc2023.subsystems.lights.LightsIO;
// import org.first5924.frc2023.subsystems.lights.LightsIOReal;
// import org.first5924.frc2023.subsystems.lights.LightsSubsystem;
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
  private final DriveSubsystem mDrive;
  private final TelescopeSubsystem mTelescope;
  private final PivotSubsystem mPivot;
  private final GrabberSubsystem mGrabber;
  // private final VisionSubsystem mVision;
  // private final LightsSubsystem mLights;

  private final CommandXboxController mDriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController mOperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  // private final LoggedDashboardChooser<Alliance> mAllianceChooser = new LoggedDashboardChooser<>("AllianceChooser");
  private final LoggedDashboardChooser<AutoRoutines> mAutoChooser = new LoggedDashboardChooser<>("AutoChooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.kCurrentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        mDrive = new DriveSubsystem(new DriveIOSparkMax());
        mTelescope = new TelescopeSubsystem(new TelescopeIOTalonFX());
        mPivot = new PivotSubsystem(new PivotIOSparkMax());
        mGrabber = new GrabberSubsystem(new GrabberIOSparkMax());
        // mVision = new VisionSubsystem(new VisionIOReal());
        // mLights = new LightsSubsystem(new LightsIOReal());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        mDrive = new DriveSubsystem(new DriveIO() {});
        mTelescope = new TelescopeSubsystem(new TelescopeIO() {});
        mPivot = new PivotSubsystem(new PivotIO() {});
        mGrabber = new GrabberSubsystem(new GrabberIO() {});
        // mVision = new VisionSubsystem(new VisionIO() {});
        // mLights = new LightsSubsystem(new LightsIO() {});
        break;

      // Replayed robot, disable IO implementations
      default:
        mDrive = new DriveSubsystem(new DriveIO() {});
        mTelescope = new TelescopeSubsystem(new TelescopeIO() {});
        mPivot = new PivotSubsystem(new PivotIO() {});
        mGrabber = new GrabberSubsystem(new GrabberIO() {});
        // mVision = new VisionSubsystem(new VisionIO() {});
        // mLights = new LightsSubsystem(new LightsIO() {});
        break;
    }

    // mAllianceChooser.addDefaultOption("Blue", Alliance.Blue);
    // mAllianceChooser.addOption("Red", Alliance.Red);

    mAutoChooser.addDefaultOption("One Piece Over Climb", AutoRoutines.onePieceOverClimb);
    // mAutoChooser.addOption("One Piece Around Climb", AutoRoutines.onePieceAroundClimb);
    mAutoChooser.addOption("One Piece Mobility", AutoRoutines.onePieceMobility);
    mAutoChooser.addOption("One Piece Stationary", AutoRoutines.onePieceStationary);
    mAutoChooser.addOption("Nothing", AutoRoutines.nothing);

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
    // Driver Left Stick and Driver Right Stick
    mDrive.setDefaultCommand(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    // Driver Left Bumper
    mDriverController.leftBumper().whileTrue(new TurnInPlace(mDrive, mDriverController::getLeftY, mDriverController::getRightX));
    // Driver Right Bumper
    mDriverController.rightBumper().whileTrue(new CurvatureDrive(mDrive, mDriverController::getLeftY, mDriverController::getRightX, 0.25));
    // Driver Left Bumper and Right Bumper
    mDriverController.leftBumper().and(mDriverController.rightBumper()).whileTrue(new TurnInPlace(mDrive, mDriverController::getLeftY, mDriverController::getRightX, 0.25));

    // Operator Left Stick
    mPivot.setDefaultCommand(new RotatePivot(mPivot, mOperatorController::getLeftY));
    // Operator Right Stick
    mTelescope.setDefaultCommand(new ExtendAndRetractTelescope(mTelescope, mOperatorController::getRightY));
    // Operator Left Trigger
    mOperatorController.leftTrigger().whileTrue(new Release(mGrabber));
    // Operator Right Trigger
    mOperatorController.rightTrigger().whileTrue(new Grab(mGrabber));
    // Operator Left Bumper
    mOperatorController.leftBumper().whileTrue(new SlowRelease(mGrabber));

    mOperatorController.x().onTrue(new SetTelescope(mTelescope, mOperatorController::getRightY, 6));
    
    mOperatorController.y().onTrue(new SetPivot(mPivot, mOperatorController::getLeftY, 180));
    //Set pivot for substation/grid
    mOperatorController.pov(35).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kDoubleSubstation));
    mOperatorController.pov(90).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kSingleSubstation));
    mOperatorController.pov(135).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kMiddleGridCube));
    mOperatorController.pov(180).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kTopGridCube));
    mOperatorController.pov(225).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kMiddleGridCone));
    mOperatorController.pov(0).whileTrue(new SetPivot(mPivot, mOperatorController::getLeftY, PivotConstants.kGroundPickup));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    switch (mAutoChooser.get()) {
      case onePieceOverClimb:
        return new OnePieceOverClimbAuto(mDrive, mPivot, mGrabber, mTelescope);
      // case onePieceAroundClimb:
      //   return new OnePieceAroundClimbAuto(mDrive, mPivot, mGrabber, mTelescope, mAllianceChooser.get());
      case onePieceMobility:
        return new OnePieceMobilityAuto(mDrive, mPivot, mGrabber, mTelescope);
      case onePieceStationary:
        return new OnePieceStationaryAuto(mPivot, mGrabber, mTelescope);
      case nothing:
        return new NothingAuto(mPivot, mTelescope);
      default:
        return new NothingAuto(mPivot, mTelescope);
    }
  }
}
