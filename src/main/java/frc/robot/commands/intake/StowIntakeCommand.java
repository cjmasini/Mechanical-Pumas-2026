package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Command to stow the intake.
 */
public class StowIntakeCommand extends Command {

    private final LoadSubsystem loadSubsystem;

    public StowIntakeCommand(LoadSubsystem loadSubsystem) {
        this.loadSubsystem = loadSubsystem;
        addRequirements(loadSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return loadSubsystem.stowIntake();
    }

    @Override
    public void end(boolean interrupted) {
        loadSubsystem.cancel();
    }
}
