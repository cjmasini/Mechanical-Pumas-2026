package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command that bucks the intake to dislodge stuck fuel
 */
public class BuckCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;
    private boolean buckingIntake = false;

    /**
     * Creates a new ShootAndBuckCommand.
     *
     * @param intakeSubsystem The intake subsystem to control for bucking
     */
    public BuckCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        buckingIntake = true;
        intakeSubsystem.setIntakeSpeed(1);
    }

    @Override
    public void execute() {
        if (buckingIntake) {
            if (intakeSubsystem.buckIntake()) {
                buckingIntake = false;
            }
        } else {
            intakeSubsystem.deployIntake();
        }
    }

    @Override
    public boolean isFinished() {
        return !buckingIntake && intakeSubsystem.isDeployed();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.cancel();
    }
}
