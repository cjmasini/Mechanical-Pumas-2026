package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Spins launch motor at a set speed
 */
public class BasicLaunchCommand extends Command {

    private final LaunchSubsystem launchSubsystem;

    /**
     * Creates a new LaunchCommand.
     */
    public BasicLaunchCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;

        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize() {
        this.launchSubsystem.setLauncherSpeed(.55);
    }


    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        this.launchSubsystem.cancel();
    }
}
