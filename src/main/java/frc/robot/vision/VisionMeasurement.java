package frc.robot.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

public class VisionMeasurement {
    public final Pose2d pose;
    public final double timestampSeconds;
    public final Matrix<N3, N1> stdDevs;
    public final int tagCount;
    public final double avgTagDist;

    public VisionMeasurement(
            Pose2d pose,
            double timestampSeconds,
            Matrix<N3, N1> stdDevs,
            int tagCount,
            double avgTagDist) {
        this.pose = pose;
        this.timestampSeconds = timestampSeconds;
        this.stdDevs = stdDevs;
        this.tagCount = tagCount;
        this.avgTagDist = avgTagDist;
    }
}
