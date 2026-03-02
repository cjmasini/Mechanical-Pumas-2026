package frc.robot.subsystems;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LimelightCamera;
import frc.robot.vision.VisionCamera;
import frc.robot.vision.VisionMeasurement;

/**
 * VisionSubsystem controls generic VisionCameras (for now just Limelight is
 * implemented)
 * and is responsible for turning their outputs into robot pose measurements.
 */
public class VisionSubsystem extends SubsystemBase {

    private final DriveSubsystem driveSubsystem;

    // All vision cameras
    private final VisionCamera[] cameras;

    /**
     * Constructs a VisionSubsystem that manages all vision cameras.
     *
     * @param driveSubsystem The drivetrain subsystem
     * 
     */
    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        this.cameras = new VisionCamera[] {
                new LimelightCamera(VisionConstants.FRONT_RIGHT_CAMERA_NAME,
                        VisionConstants.FRONT_RIGHT_LIMELIGHT_TRANSFORM)
        };
    }

    @Override
    public void periodic() {
        for (VisionCamera camera : cameras) {
            Optional<VisionMeasurement> measurement = camera.getEstimatedGlobalPose();
            if (measurement.isEmpty()) {
                continue;
            }

            VisionMeasurement m = measurement.get();
            double measurementAge = Timer.getFPGATimestamp() - m.timestampSeconds;
            // Skip old measurements
            if (measurementAge > 0.5) {
                continue;
            }
            logMeasurement(camera, m);

            driveSubsystem.addVisionMeasurement(m.pose, m.timestampSeconds, m.stdDevs);
        }
    }

    /**
     * Logs a vision measurement to SmartDashboard for debugging.
     */
    private void logMeasurement(VisionCamera camera, VisionMeasurement m) {
        String prefix = "Vision/" + camera.getName();

        SmartDashboard.putNumber(prefix + "/pose_x", m.pose.getX());
        SmartDashboard.putNumber(prefix + "/pose_y", m.pose.getY());
        SmartDashboard.putNumber(prefix + "/pose_rot_deg",
                m.pose.getRotation().getDegrees());

        SmartDashboard.putNumber(prefix + "/timestamp", m.timestampSeconds);
        SmartDashboard.putNumber(prefix + "/tagCount", m.tagCount);
        SmartDashboard.putNumber(prefix + "/avgTagDist", m.avgTagDist);

        // Std devs: x, y, theta
        SmartDashboard.putNumber(prefix + "/std_x", m.stdDevs.get(0, 0));
        SmartDashboard.putNumber(prefix + "/std_y", m.stdDevs.get(1, 0));
        SmartDashboard.putNumber(prefix + "/std_theta", m.stdDevs.get(2, 0));

        // Vision vs odometry error (shows correction magnitude)
        Pose2d odomPose = driveSubsystem.getPose();
        SmartDashboard.putNumber(prefix + "/error_x", m.pose.getX() - odomPose.getX());
        SmartDashboard.putNumber(prefix + "/error_y", m.pose.getY() - odomPose.getY());
        SmartDashboard.putNumber(prefix + "/error_rot_deg",
                m.pose.getRotation().minus(odomPose.getRotation()).getDegrees());

        // Pose as array for AdvantageScope
        SmartDashboard.putNumberArray(prefix + "/pose",
                new double[] { m.pose.getX(), m.pose.getY(), m.pose.getRotation().getRadians() });
    }

    /**
     * Expose cameras for debugging / future features if needed.
     */
    public VisionCamera[] getCameras() {
        return cameras;
    }
}
