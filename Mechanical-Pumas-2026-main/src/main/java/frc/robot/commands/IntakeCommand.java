package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Spins intake motor at a set speed
 */
public class IntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    /**
     * Creates a new IntakeCommand.
     */
    public IntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;

        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        this.intakeSubsystem.setIntakeSpeed(.55);
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
        this.intakeSubsystem.cancel();
    }
}
