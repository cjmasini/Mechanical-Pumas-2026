package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that spins up the flywheel based on distance and waits until it reaches target velocity.
 * Finishes when flywheel is at target velocity.
 */
public class DynamicLaunchCommand extends Command {

    private final LaunchSubsystem launchSubsystem;
    private final VisionSubsystem visionSubsystem;

    public DynamicLaunchCommand(LaunchSubsystem launchSubsystem, VisionSubsystem visionSubsystem) {
        this.launchSubsystem = launchSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize() {
        double distance = visionSubsystem.getDistanceToHub();
        launchSubsystem.setLauncherVelocityForDistance(distance);
    }

    @Override
    public void execute() {
        double distance = visionSubsystem.getDistanceToHub();
        launchSubsystem.setLauncherVelocityForDistance(distance);
    }

    @Override
    public boolean isFinished() {
        return launchSubsystem.isAtTargetVelocity();
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop the launcher - we want it to keep spinning for the next command
    }
}
