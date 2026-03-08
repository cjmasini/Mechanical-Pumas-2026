package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that enables distance-based automatic launcher velocity control.
 * The launcher continuously adjusts velocity based on distance to the target hub.
 *
 * In order to shoot::
 * - Distance must be within supplied calibrated range
 * - Robot must be facing the hub
 *
 * If checks fail, the launcher does nothing
 */
public class AutoShootCommand extends Command {

    private final LaunchSubsystem launchSubsystem;
    private final VisionSubsystem visionSubsystem;

    // TODO: Move to constants once finalized
    private static final double FACING_TOLERANCE_DEGREES = 2.0;

    /**
     * Creates a new AutoShootCommand.
     *
     * @param launchSubsystem The launch subsystem to control
     * @param visionSubsystem The vision subsystem for targeting info
     */
    public AutoShootCommand(LaunchSubsystem launchSubsystem, VisionSubsystem visionSubsystem) {
        this.launchSubsystem = launchSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(launchSubsystem);
    }

    @Override
    public void execute() {
        double distance = visionSubsystem.getDistanceToHub();

        if (distance < launchSubsystem.getMinDistance() ||
            distance > launchSubsystem.getMaxDistance()) {
            return;
        }

        if (!visionSubsystem.isFacingHub(FACING_TOLERANCE_DEGREES)) {
            return;
        }

        launchSubsystem.setLauncherVelocityForDistance(distance);
    }

    @Override
    public void end(boolean interrupted) {
        launchSubsystem.cancel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}