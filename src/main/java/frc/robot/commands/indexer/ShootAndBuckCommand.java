package frc.robot.commands.indexer;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LoadSubsystem;

/**
 * Command that runs conveyor and loader motors at full speed continuously.
 * After a delay, also starts bucking the intake while continuing to run the motors.
 * Runs until canceled.
 */
public class ShootAndBuckCommand extends Command {

    private final LoadSubsystem loadSubsystem;
    private final double BUCK_DELAY_TIME = 3.0;
    private double startTime;
    private boolean movingToStowed;

    /**
     * Creates a new ShootAndBuckCommand.
     *
     * @param loadSubsystem The load subsystem to control
     * @param BUCK_DELAY_TIME Delay before starting to buck the intake
     */
    public ShootAndBuckCommand(LoadSubsystem loadSubsystem) {
        this.loadSubsystem = loadSubsystem;
        addRequirements(loadSubsystem);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        movingToStowed = false;

        // Start conveyor and loaders at full speed
        loadSubsystem.setConveyorSpeed(1.0);
        loadSubsystem.setLoaderSpeed(1.0);
    }

    @Override
    public void execute() {
        // Keep conveyor and loaders running at full speed
        loadSubsystem.setConveyorSpeed(1.0);
        loadSubsystem.setLoaderSpeed(1.0);

        // After delay, also buck the intake
        if (Timer.getFPGATimestamp() - startTime >= BUCK_DELAY_TIME) {
            if (movingToStowed) {
                if (loadSubsystem.stowIntake()) {
                    movingToStowed = false;
                }
            } else {
                if (loadSubsystem.buckIntake()) {
                    movingToStowed = true;
                }
            }
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        loadSubsystem.cancel();
    }
}
