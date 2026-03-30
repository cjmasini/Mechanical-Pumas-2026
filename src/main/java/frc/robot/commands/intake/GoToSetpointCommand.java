package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Command that moves the deploy mechanism to the SmartDashboard setpoint.
 * Useful for tuning and testing positions.
 */
public class GoToSetpointCommand extends Command {

    private final IntakeSubsystem intakeSubsystem;

    public GoToSetpointCommand(IntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        return intakeSubsystem.goToSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubsystem.cancel();
    }
}
