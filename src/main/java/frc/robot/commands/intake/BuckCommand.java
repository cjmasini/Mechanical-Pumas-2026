package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command that bucks the intake to dislodge stuck fuel
 */
public class BuckCommand extends SequentialCommandGroup {


    /**
     * Creates a new ShootAndBuckCommand.
     * Starts intake, moves to buck position, deploys intake, then stops intake
     *
     * @param intakeSubsystem The intake subsystem to control for bucking
     */
    public BuckCommand(IntakeSubsystem intakeSubsystem) {
        addRequirements(intakeSubsystem);
        this.addCommands(new IntakeCommand(intakeSubsystem), new BuckIntakeCommand(intakeSubsystem), new DeployIntakeCommand(intakeSubsystem), new InstantCommand(() -> intakeSubsystem.setIntakeSpeed(0), intakeSubsystem));
    }
}
