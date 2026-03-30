package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.drive.AimAtHubCommand;
import frc.robot.commands.drive.HoldPositionCommand;
import frc.robot.commands.indexer.LoadCommand;
import frc.robot.commands.intake.BuckCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.LoadSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Basic auto shoot command that uses a fixed velocity setpoint from SmartDashboard.
 * First aims at the hub, then holds position while spinning up to the dashboard velocity
 * and firing. After 3 seconds of firing, starts bucking the intake.
 * Runs until canceled.
 */
public class BasicAutoShootCommand extends SequentialCommandGroup {

    public BasicAutoShootCommand(DriveSubsystem driveSubsystem, LaunchSubsystem launchSubsystem,
            LoadSubsystem loadSubsystem, IntakeSubsystem intakeSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            new AimAtHubCommand(driveSubsystem, visionSubsystem),
            new ParallelCommandGroup(
                new HoldPositionCommand(driveSubsystem),
                new VelocityLaunchCommand(launchSubsystem),
                new SequentialCommandGroup(
                    new WaitCommand(1),
                    new LoadCommand(loadSubsystem),
                    new WaitCommand(3),
                    // Start bucking after 3 seconds
                    new BuckCommand(intakeSubsystem).withTimeout(3)
                )
            )
        );
    }
}
