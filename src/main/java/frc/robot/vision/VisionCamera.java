package frc.robot.vision;

import java.util.List;
import java.util.Optional;

public interface VisionCamera {
    /**
     * Feeds the robot's current gyro orientation and yaw rate to the camera
     * (required for MegaTag 2 / heading-constrained pose estimation).
     *
     * @param yawDegrees       Field-relative robot yaw in WPILib Blue-alliance coordinates (degrees)
     * @param yawRateDegPerSec Robot yaw rate (degrees per second)
     */
    default void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {}

    /** Returns the latest new vision measurement if available this loop. */
    Optional<VisionMeasurement> getEstimatedGlobalPose();

    /**
     * Returns all new vision measurements available since the last loop.
     * Single-result cameras (like Limelight) use this default wrapper; queued cameras
     * (like PhotonVision) override this to drain all unread frames without dropping data.
     */
    default List<VisionMeasurement> getEstimatedGlobalPoses() {
        return getEstimatedGlobalPose().map(List::of).orElse(List.of());
    }

    /** For logging / debugging. */
    String getName();
}
