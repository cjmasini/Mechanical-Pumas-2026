package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.VisionCamera;

public class AutoTargetHubCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;
    private final CommandXboxController driverXbox;

    public AutoTargetHubCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, CommandXboxController driverXbox) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.driverXbox = driverXbox;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double yMovement = driverXbox.getLeftY();
        double xMovement = driverXbox.getLeftX();
        double processedY = -MathUtil.applyDeadband(Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND);
        double processedX = -MathUtil.applyDeadband(Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND);

        boolean hasVisionTarget = hasValidVisionMeasurement();

        if (hasVisionTarget) {
            Rotation2d angleToHub = visionSubsystem.getFieldRelativeAngleToHub();

            driveSubsystem.driveAndOrient(processedY, processedX, angleToHub.getDegrees(), true);
        } else {
            double manualRot = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND);
            driveSubsystem.drive(processedY, processedX, manualRot, true);
        }
    }

    private boolean hasValidVisionMeasurement() {
        for (VisionCamera camera : visionSubsystem.getCameras()) {
            if (camera.getEstimatedGlobalPose().isPresent()) {
                return true;
            }
        }
        return false;
    }
}