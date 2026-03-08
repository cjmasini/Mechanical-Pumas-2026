package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.drive.AimAtHubCommand;
import frc.robot.commands.drive.HoldPositionCommand;
import frc.robot.commands.indexer.ShootAndBuckCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.LoadSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that performs an automatic shooting sequence.
 * First aims at the hub, then holds that position while spinning up the flywheel
 * and firing. After 3 seconds of firing, starts bucking the intake while continuing
 * to run the conveyor and loaders.
 * Runs until canceled.
 */
public class AutoShootCommand extends SequentialCommandGroup {

    public AutoShootCommand(DriveSubsystem driveSubsystem, LaunchSubsystem launchSubsystem,
            LoadSubsystem loadSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            new AimAtHubCommand(driveSubsystem, visionSubsystem),
            new ParallelCommandGroup(
                new HoldPositionCommand(driveSubsystem),
                new SequentialCommandGroup(
                    new DynamicLaunchCommand(launchSubsystem, visionSubsystem),
                    new ShootAndBuckCommand(loadSubsystem)
                )
            )
        );
    }
}
