package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Spins load motor at a set speed
 */
public class LoadersOnlyCommand extends Command {

    private final LoadSubsystem loadSubsystem;

    /**
     * Creates a new LoadCommand.
     */
    public LoadersOnlyCommand(LoadSubsystem loadSubsystem) {
        this.loadSubsystem = loadSubsystem;

        addRequirements(loadSubsystem);
    }

    @Override
    public void initialize() {
        this.loadSubsystem.setLoaderSpeed(.55);
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
        this.loadSubsystem.cancel();
    }
}
