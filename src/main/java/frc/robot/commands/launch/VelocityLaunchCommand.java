package frc.robot.commands.launch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Command that sets the launcher to a target velocity from SmartDashboard.
 * Finishes when the launcher reaches the target velocity.
 */
public class VelocityLaunchCommand extends Command {

    private final LaunchSubsystem launchSubsystem;
    private static final String VELOCITY_KEY = "Launcher/TargetRPM";
    private static final double DEFAULT_VELOCITY = -6500.0;
    /**
     * Creates a new VelocityLaunchCommand.
     *
     * @param launchSubsystem The launch subsystem to control
     */
    public VelocityLaunchCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;
        addRequirements(launchSubsystem);

        // Initialize the dashboard value if not present
        SmartDashboard.putNumber(VELOCITY_KEY,
            SmartDashboard.getNumber(VELOCITY_KEY, DEFAULT_VELOCITY));
    }

    @Override
    public void initialize() {
        double targetVelocity = SmartDashboard.getNumber(VELOCITY_KEY, DEFAULT_VELOCITY);
        launchSubsystem.setLauncherVelocity(targetVelocity);
    }

    @Override
    public void execute() {
        // Continuously update in case dashboard value changes
        double targetVelocity = SmartDashboard.getNumber(VELOCITY_KEY, DEFAULT_VELOCITY);
        launchSubsystem.setLauncherVelocity(targetVelocity);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Don't stop the launcher - keep spinning for the next command
    }
}
