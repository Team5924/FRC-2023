// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023.robot;

import org.first5924.frc2023.commands.autonomous.ThreePieceAuto;
import org.first5924.frc2023.commands.drive.CurvatureDrive;
import org.first5924.frc2023.commands.drive.TurnInPlace;
import org.first5924.frc2023.commands.telescope.ExtendAndRetractTelescope;
import org.first5924.frc2023.constants.OIConstants;
import org.first5924.frc2023.constants.RobotConstants;
import org.first5924.frc2023.subsystems.drive.DriveIO;
import org.first5924.frc2023.subsystems.drive.DriveIOSparkMax;
import org.first5924.frc2023.subsystems.drive.DriveSubsystem;
import org.first5924.frc2023.subsystems.telescope.TelescopeIO;
import org.first5924.frc2023.subsystems.telescope.TelescopeIOSparkMax;
import org.first5924.frc2023.subsystems.telescope.TelescopeSubsystem;
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
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem mDrive;
  private final TelescopeSubsystem mTelescope;
  // private final PivotSubsystem mPivot = new PivotSubsystem();

  private final CommandXboxController mDriverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController mOperatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  private final LoggedDashboardChooser<Alliance> mAutoChooser = new LoggedDashboardChooser<>("AutoChooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.kCurrentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        mDrive = new DriveSubsystem(new DriveIOSparkMax());
        mTelescope = new TelescopeSubsystem(new TelescopeIOSparkMax());
        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        mDrive = new DriveSubsystem(new DriveIO() {
        });
        mTelescope = new TelescopeSubsystem(new TelescopeIO() {
        });
        break;

      // Replayed robot, disable IO implementations
      default:
        mDrive = new DriveSubsystem(new DriveIO() {
        });
        mTelescope = new TelescopeSubsystem(new TelescopeIO() {
        });
        break;
    }

    mAutoChooser.addDefaultOption("Blue", Alliance.Blue);
    mAutoChooser.addOption("Red", Alliance.Red);

    // mPivot.setDefaultCommand(new RotatePivot(mPivot, mOperatorController::getRightY));

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
    mTelescope.setDefaultCommand(new ExtendAndRetractTelescope(mTelescope, mOperatorController::getRightY));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new ThreePieceAuto(mDrive, mAutoChooser.get());
  }
}
