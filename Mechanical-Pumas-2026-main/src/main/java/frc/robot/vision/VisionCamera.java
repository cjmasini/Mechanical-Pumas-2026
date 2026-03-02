package frc.robot.vision;

import java.util.Optional;

public interface VisionCamera {
    /** Returns a new vision measurement if available this loop. */
    Optional<VisionMeasurement> getEstimatedGlobalPose();
  
    /** For logging / debugging. */
    String getName();
  }
  