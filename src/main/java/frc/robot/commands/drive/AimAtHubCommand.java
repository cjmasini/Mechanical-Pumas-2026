package frc.robot.commands.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * Command that rotates the robot to face the hub.
 * Finishes when the robot is aimed within tolerance.
 * Works using pose estimate - accuracy depends on recent vision measurements.
 */
public class AimAtHubCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private static final double ANGLE_TOLERANCE_DEGREES = 2.0;
    private boolean hasWarnedNoVision = false;

    public AimAtHubCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        hasWarnedNoVision = false;

        // Warn if no recent vision - aiming will use potentially drifted odometry
        if (!visionSubsystem.hasRecentVision()) {
            DriverStation.reportWarning("AimAtHubCommand: No recent vision data - using odometry estimate", false);
            hasWarnedNoVision = true;
        }
    }

    @Override
    public void execute() {
        // Publish status for driver awareness
        SmartDashboard.putBoolean("AutoShoot/VisionActive", visionSubsystem.hasRecentVision());

        // Warn once if vision is lost during aiming
        if (!hasWarnedNoVision && !visionSubsystem.hasRecentVision()) {
            DriverStation.reportWarning("AimAtHubCommand: Lost vision during aim", false);
            hasWarnedNoVision = true;
        }

        // Get the angle to the hub and calculate target heading
        double currentHeading = driveSubsystem.getHeading();
        double angleToHub = visionSubsystem.getAngleToHub().getDegrees();
        double targetHeading = currentHeading + angleToHub;

        // Drive with zero translation while orienting to face the hub
        driveSubsystem.driveAndOrient(0, 0, targetHeading);
    }

    @Override
    public boolean isFinished() {
        // Finished when angle to hub is within tolerance
        return Math.abs(visionSubsystem.getAngleToHub().getDegrees()) < ANGLE_TOLERANCE_DEGREES;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop driving
        driveSubsystem.drive(0, 0, 0, true);
    }
}
