package frc.robot.commands.drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Command that holds the robot's position and heading from when the command started.
 * Actively fights against being pushed by using PID control on X, Y, and heading.
 * Runs until canceled.
 */
public class HoldPositionCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private Pose2d targetPose;

    // PID controllers for position hold
    private final PIDController xController;
    private final PIDController yController;

    // Position tolerance in meters - stops driving when within this distance
    private static final double POSITION_TOLERANCE = 0.02;

    public HoldPositionCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        // TODO: Tune
        xController = new PIDController(2.0, 0, 0.1);
        yController = new PIDController(2.0, 0, 0.1);

        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        targetPose = driveSubsystem.getPose();

        xController.setSetpoint(targetPose.getX());
        yController.setSetpoint(targetPose.getY());
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double xSpeed = xController.calculate(currentPose.getX());
        double ySpeed = yController.calculate(currentPose.getY());

        xSpeed = clamp(xSpeed, -0.75, 0.75);
        ySpeed = clamp(ySpeed, -0.75, 0.75);

        driveSubsystem.driveAndOrient(xSpeed, ySpeed, targetPose.getRotation().getDegrees());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.setX();
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
