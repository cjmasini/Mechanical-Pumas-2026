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
 * Basic auto shoot command that uses a fixed velocity setpoint from SmartDashboard.
 * First aims at the hub, then holds position while spinning up to the dashboard velocity
 * and firing. After 3 seconds of firing, starts bucking the intake.
 * Runs until canceled.
 */
public class BasicAutoShootCommand extends SequentialCommandGroup {

    public BasicAutoShootCommand(DriveSubsystem driveSubsystem, LaunchSubsystem launchSubsystem,
            LoadSubsystem loadSubsystem, VisionSubsystem visionSubsystem) {
        addCommands(
            // First, aim at the hub
            new AimAtHubCommand(driveSubsystem, visionSubsystem),
            // Then hold position while shooting
            new ParallelCommandGroup(
                // Hold position (captures pose after aiming, fights being pushed)
                new HoldPositionCommand(driveSubsystem),
                // Shooting sequence
                new SequentialCommandGroup(
                    // Spin up to dashboard velocity and wait until ready
                    new VelocityLaunchCommand(launchSubsystem),
                    // Fire continuously, start bucking after 3 seconds
                    new ShootAndBuckCommand(loadSubsystem)
                )
            )
        );
    }
}
