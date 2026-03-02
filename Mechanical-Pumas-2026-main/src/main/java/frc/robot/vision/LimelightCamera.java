package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

public class LimelightCamera implements VisionCamera {
    private final String limelightName;

    // Current std devs used for the last measurement
    private Matrix<N3, N1> curStdDevs = SINGLE_TAG_STD_DEVS; // default

    // Heuristic std devs for different scenarios - stolen from K9.0 for now
    private static final Matrix<N3, N1> SINGLE_TAG_STD_DEVS = VecBuilder.fill(4, 4, 8);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1);

    public LimelightCamera(String limelightName, Transform3d transform) {
        this.limelightName = limelightName;

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                transform.getX(), // forward (+X)
                transform.getY(), // left (+Y)
                transform.getZ(), // up (+Z)
                Math.toDegrees(transform.getRotation().getX()), // roll
                Math.toDegrees(transform.getRotation().getY()), // pitch
                Math.toDegrees(transform.getRotation().getZ()) // yaw
        );
    }

    @Override
    public Optional<VisionMeasurement> getEstimatedGlobalPose() {
        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue(limelightName);

        if (est == null || !LimelightHelpers.validPoseEstimate(est)) {
            curStdDevs = SINGLE_TAG_STD_DEVS;
            return Optional.empty();
        }

        // Update curStdDevs based on tagCount and avgTagDist
        updateStdDevs(est);

        VisionMeasurement measurement = new VisionMeasurement(
                est.pose, // Pose2d in WPILib blue frame
                est.timestampSeconds, // already latency-corrected
                curStdDevs,
                est.tagCount,
                est.avgTagDist);

        return Optional.of(measurement);
    }

    @Override
    public String getName() {
        return limelightName;
    }

    /**
     * Heuristic to determine how much to trust this measurement:
     * - If no tags / bad distance: fall back to single-tag defaults
     * - If single tag and very far: effectively ignore (huge std devs)
     * - If multi-tag: start from tighter base, then scale with distance
     */
    private void updateStdDevs(PoseEstimate est) {
        int tagCount = est.tagCount;
        double avgDist = est.avgTagDist; // meters

        // Safe default
        Matrix<N3, N1> stdDevs = SINGLE_TAG_STD_DEVS;

        // If no tags or bogus distance, just keep conservative noise
        if (tagCount <= 0 || avgDist <= 0) {
            curStdDevs = stdDevs;
            return;
        }

        // Multi-tag: start from tighter base
        if (tagCount > 1) {
            stdDevs = MULTI_TAG_STD_DEVS;
        }

        // Single-tag very far away: don't trust it at all
        if (tagCount == 1 && avgDist > 4.0) {
            curStdDevs = VecBuilder.fill(
                    Double.MAX_VALUE,
                    Double.MAX_VALUE,
                    Double.MAX_VALUE);
            return;
        }

        // Scale noise with distance so farther tags are less trusted
        double scale = 1.0 + (avgDist * avgDist / 30.0);
        curStdDevs = stdDevs.times(scale);
    }
}
