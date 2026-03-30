package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to stow the intake.
 */
public class StowIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public StowIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.stowIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.stowIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.cancel();
    }
}
