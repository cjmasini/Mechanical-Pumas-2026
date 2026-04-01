package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to stow the intake.
 */
public class BuckIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public BuckIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.buckIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.buckIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.cancel();
    }
}
