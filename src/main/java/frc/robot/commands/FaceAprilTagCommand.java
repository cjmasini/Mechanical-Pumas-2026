package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command to drive the robot while automatically orienting towards a detected
 * AprilTag.
 * Uses the DriveSubsystem's driveWhileTargeting check.
 */
public class FaceAprilTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController driverXbox;

    public FaceAprilTagCommand(DriveSubsystem driveSubsystem, CommandXboxController driverXbox) {
        this.driveSubsystem = driveSubsystem;
        this.driverXbox = driverXbox;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double yMovement = driverXbox.getLeftY();
        double xMovement = driverXbox.getLeftX();

        double processedY = -MathUtil.applyDeadband(Math.pow(yMovement, 3), OperatorConstants.DRIVE_DEADBAND);
        double processedX = -MathUtil.applyDeadband(Math.pow(xMovement, 3), OperatorConstants.DRIVE_DEADBAND);

        double processedRot = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND);

        driveSubsystem.driveWhileTargeting(processedY, processedX, processedRot);
    }
}
