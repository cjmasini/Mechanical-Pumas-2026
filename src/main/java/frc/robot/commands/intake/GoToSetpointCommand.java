package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Command that moves the deploy mechanism to the SmartDashboard setpoint.
 * Useful for tuning and testing positions.
 */
public class GoToSetpointCommand extends Command {

    private final LoadSubsystem loadSubsystem;

    public GoToSetpointCommand(LoadSubsystem loadSubsystem) {
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
        return loadSubsystem.goToSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        loadSubsystem.cancel();
    }
}
