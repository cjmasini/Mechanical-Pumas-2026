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
import frc.robot.commands.FaceAprilTagCommand;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.BasicLaunchCommand;
import frc.robot.commands.BasicLoadCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.MoveCommand;
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
  @SuppressWarnings("unused")
  private final VisionSubsystem visionSubsystem;
  private final LaunchSubsystem launchSubsystem;
  private final LoadSubsystem loadSubsystem;
  private final IntakeSubsystem intakeSubsystem;

  private final CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, IO devices, and commands.
   */
  public RobotContainer() {
    // Initialize all subsystems for the robot here
    driveSubsystem = new DriveSubsystem();
    visionSubsystem = new VisionSubsystem(driveSubsystem);
    launchSubsystem = new LaunchSubsystem();
    loadSubsystem = new LoadSubsystem();
    intakeSubsystem = new IntakeSubsystem();

    // Choosers show up on our dashboard and let us modify options before and during
    // matches
    autoChooser = AutoBuilder.buildAutoChooser();

    // Setup all controller button mappings
    configureBindings();

    // For choosers (or other info) to show up on the dashboard, they must be added
    SmartDashboard.putData(autoChooser);
  }

  /**
   * Configure the trigger bindings for the robot.
   * 
   * In this function, we initialize commands and map them to controller buttons
   */
  private void configureBindings() {
    // Some subsystems have default commands that run when no other command is using
    // them
    // Our move command controls how the robot moves around based on controller
    // input
    MoveCommand moveCommand = new MoveCommand(this.driveSubsystem, this.driverXbox);
    driveSubsystem.setDefaultCommand(moveCommand);

    driverXbox.x().onTrue(
        driveSubsystem.createDriveToPoseCommand(
            new Pose2d(2.8, 4.2, Rotation2d.fromDegrees(0.0))));

    // Initialize example command and set it to run when the A button is pressed
    BasicLaunchCommand launchCommand = new BasicLaunchCommand(this.launchSubsystem);
    driverXbox.a().and(driverXbox.rightTrigger().negate()).onTrue(launchCommand);
    // Initialize load command and set it to run when the Y button is pressed
    BasicLoadCommand loadCommand = new BasicLoadCommand(this.loadSubsystem);
    driverXbox.y().onTrue(loadCommand);
    // Initialize intake command and set it to run when the Right Bumper is pressed
    IntakeCommand intakeCommand = new IntakeCommand(this.intakeSubsystem);
    driverXbox.rightBumper().onTrue(intakeCommand);
    // Adding commands as named commands allows PathPlanner to use them in autos
    NamedCommands.registerCommand("launchCommand", launchCommand);

    // All subsystems (other than drive) should be passed in to the cancel command
    CancelCommand cancelCommand = new CancelCommand(
        List.of(launchSubsystem));
    driverXbox.rightTrigger().onTrue(cancelCommand);

    // Instant commands are useful for simple one-time actions
    // Manually re-zero the gyro if robot is misaligned using an instant command
    InstantCommand resetGyro = new InstantCommand(() -> this.driveSubsystem.zeroHeading());
    driverXbox.rightStick().and(driverXbox.leftStick()).onTrue(resetGyro);

    // Bind AutoTarget to Left Trigger
    // While held, the robot will auto-orient to any visible AprilTag while allowing
    // driving
    driverXbox.leftTrigger().whileTrue(new FaceAprilTagCommand(driveSubsystem, driverXbox));
  }

  // Return the auto selected on the dashboard
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  public DriveSubsystem getDriveSubsystem() {
    return driveSubsystem;
  }
}