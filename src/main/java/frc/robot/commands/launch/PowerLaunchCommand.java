package frc.robot.commands.launch;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LaunchSubsystem;

/**
 * Spins launch motor at a set speed
 */
public class PowerLaunchCommand extends Command {

    private final LaunchSubsystem launchSubsystem;

    public PowerLaunchCommand(LaunchSubsystem launchSubsystem) {
        this.launchSubsystem = launchSubsystem;

        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.putNumber("launch/power",
            SmartDashboard.getNumber("launch/power", .55));
        this.launchSubsystem.setLauncherPower(SmartDashboard.getNumber("launch/power", 0.55));
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
        this.launchSubsystem.cancel();
    }
}
