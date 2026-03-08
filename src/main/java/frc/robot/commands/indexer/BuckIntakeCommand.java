package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Command that repeatedly moves the intake between stowed and slightly open positions to dislodge fuel.
 * Runs until canceled.
 */
public class BuckIntakeCommand extends Command {

    private final LoadSubsystem loadSubsystem;
    private boolean movingToStowed;

    public BuckIntakeCommand(LoadSubsystem loadSubsystem) {
        this.loadSubsystem = loadSubsystem;
        addRequirements(loadSubsystem);
    }

    @Override
    public void initialize() {
        movingToStowed = false;
    }

    @Override
    public void execute() {
        if (movingToStowed) {
            if (loadSubsystem.stowIntake()) {
                movingToStowed = false;
            }
        } else {
            if (loadSubsystem.buckIntake()) {
                movingToStowed = true;
            }
        }
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
