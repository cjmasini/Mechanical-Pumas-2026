package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Command that runs the conveyor and loader motors at full speed for a specified duration.
 * Used for feeding game pieces into the launcher.
 */
public class RevLoadCommand extends Command {

    private final LoadSubsystem loadSubsystem;

    /**
     * Creates a new LoadCommand.
     *
     * @param loadSubsystem The load subsystem to control
     */
    public RevLoadCommand(LoadSubsystem loadSubsystem) {
        this.loadSubsystem = loadSubsystem;
        addRequirements(loadSubsystem);
    }

    @Override
    public void initialize() {
        loadSubsystem.setIndexerSpeed(-1.0);
        loadSubsystem.setLoaderSpeed(-1.0);
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
        loadSubsystem.cancel();
    }
}
