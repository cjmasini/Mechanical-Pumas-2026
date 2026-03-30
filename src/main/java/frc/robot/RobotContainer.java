// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoTargetHubCommand;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.drive.FaceAprilTagCommand;
import frc.robot.commands.drive.MoveCommand;
import frc.robot.commands.indexer.TestCommand;
import frc.robot.commands.indexer.LoadCommand;
import frc.robot.commands.indexer.LoadersOnlyCommand;
import frc.robot.commands.indexer.RevLoadCommand;
import frc.robot.commands.intake.BuckCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.GoToSetpointCommand;
import frc.robot.commands.intake.IntakeCommand;
import frc.robot.commands.intake.StowIntakeCommand;
import frc.robot.commands.launch.PowerLaunchCommand;
import frc.robot.commands.launch.VelocityLaunchCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.LoadSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {

  private final DriveSubsystem driveSubsystem;
  private final VisionSubsystem visionSubsystem;
  private final LaunchSubsystem launchSubsystem;
  private final LoadSubsystem loadSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  // private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Initialize all subsystems for the robot here
    driveSubsystem = new DriveSubsystem();
    launchSubsystem = new LaunchSubsystem();
    loadSubsystem = new LoadSubsystem();
    intakeSubsystem = new IntakeSubsystem();
    visionSubsystem = new VisionSubsystem(driveSubsystem);

    // Choosers show up on our dashboard and let us modify options before and during
    // matches
    // autoChooser = AutoBuilder.buildAutoChooser();

    // Setup all controller button mappings
    configureBindings();

    // For choosers (or other info) to show up on the dashboard, they must be added
    // SmartDashboard.putData(autoChooser);
  }

  /**
   * Configure the trigger bindings for the robot.
   * 
   * In this function, we initialize commands and map them to controller buttons
   */
  private void configureBindings() {
    MoveCommand moveCommand = new MoveCommand(this.driveSubsystem, this.driverXbox);
    driveSubsystem.setDefaultCommand(moveCommand);

    // driverXbox.x().onTrue(
    //     driveSubsystem.createDriveToPoseCommand(
    //         new Pose2d(2.8, 4.2, Rotation2d.fromDegrees(0.0))));

    VelocityLaunchCommand launchCommand = new VelocityLaunchCommand(this.launchSubsystem);
    driverXbox.rightBumper().and(driverXbox.rightTrigger().negate()).onTrue(launchCommand);
    NamedCommands.registerCommand("flywheel", launchCommand);
    LoadCommand loadCommand = new LoadCommand(this.loadSubsystem);
    driverXbox.rightTrigger().onTrue(loadCommand);
    NamedCommands.registerCommand("load", loadCommand);
    RevLoadCommand revLoadCommand = new RevLoadCommand(this.loadSubsystem);
    driverXbox.b().whileTrue(revLoadCommand);
    NamedCommands.registerCommand("revload", revLoadCommand);
    BuckCommand buckCommand = new BuckCommand(intakeSubsystem);
    driverXbox.a().onTrue(buckCommand);

    AutoTargetHubCommand autoTargetHubCommand = new AutoTargetHubCommand(driveSubsystem, visionSubsystem, driverXbox);
    driverXbox.start().whileTrue(autoTargetHubCommand);

    DeployIntakeCommand deployIntakeCommand = new DeployIntakeCommand(this.intakeSubsystem);
    driverXbox.x().onTrue(deployIntakeCommand);
    NamedCommands.registerCommand("deploy", deployIntakeCommand);

    StowIntakeCommand stowIntakeCommand = new StowIntakeCommand(intakeSubsystem);
    driverXbox.y().onTrue(stowIntakeCommand);
    NamedCommands.registerCommand("stow", stowIntakeCommand);

    IntakeCommand intakeCommand = new IntakeCommand(this.intakeSubsystem);
    driverXbox.leftTrigger().onTrue(intakeCommand);
    NamedCommands.registerCommand("intake", intakeCommand);

    CancelCommand cancelCommand = new CancelCommand(
        List.of(launchSubsystem, loadSubsystem, intakeSubsystem));
    driverXbox.leftBumper().onTrue(cancelCommand);
    NamedCommands.registerCommand("cancel", cancelCommand);

    InstantCommand resetGyro = new InstantCommand(() -> this.driveSubsystem.zeroHeading());
    driverXbox.rightStick().and(driverXbox.leftStick()).onTrue(resetGyro);
  }

  // Return the auto selected on the dashboard
  public Command getAutonomousCommand() {
    return new InstantCommand();//autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
 
}