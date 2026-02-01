package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.vision.VisionCamera;
import static frc.robot.Constants.RebuiltConstants.RED_HUB_CENTER;
import static frc.robot.Constants.RebuiltConstants.BLUE_HUB_CENTER;

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
        // 1. Driver Controls (Translation)
        double yMovement = driverXbox.getLeftY();
        double xMovement = driverXbox.getLeftX();
        double processedY = -MathUtil.applyDeadband(Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND);
        double processedX = -MathUtil.applyDeadband(Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND);

        // 2. Check if any vision camera has a valid pose measurement
        boolean hasVisionTarget = hasValidVisionMeasurement();

        if (hasVisionTarget) {
            // 3. Get the fused robot pose from the pose estimator (odometry + vision)
            Pose2d robotPose = driveSubsystem.getPose();

            // 4. Select Target based on Alliance
            Translation2d target = getTargetForAlliance();

            // 5. Calculate Angle to Target
            double dx = target.getX() - robotPose.getX();
            double dy = target.getY() - robotPose.getY();
            Rotation2d angleToGoal = new Rotation2d(Math.atan2(dy, dx));

            // 6. Drive while orienting towards hub
            driveSubsystem.driveAndOrient(processedY, processedX, angleToGoal.getDegrees(), true);
        } else {
            // Fallback: Manual Rotation when no vision target
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

    private Translation2d getTargetForAlliance() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            return RED_HUB_CENTER;
        }
        return BLUE_HUB_CENTER;
    }
}