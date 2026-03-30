package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command to deploy the intake.
 */
public class DeployIntakeCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public DeployIntakeCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        intakeSubsystem.deployIntake();
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.deployIntake();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.cancel();
    }
}
