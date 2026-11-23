package frc.robot.vision;

import java.util.Optional;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LimelightHelpers.PoseEstimate;

/**
 * VisionCamera implementation backed by a Limelight running MegaTag 2.
 * Filters out measurements when spinning too fast, out of field bounds,
 * or from far single tags, and scales XY standard deviations with distance.
 */
public class LimelightCamera implements VisionCamera {
    private final String limelightName;
    private double lastYawRateDegPerSec = 0.0;

    // MegaTag 2 trusts the robot gyro for heading, so theta std dev is set very large
    // while XY position trust scales with tag count and distance.
    private static final double SINGLE_TAG_XY_STD_DEV = 0.7;
    private static final double MULTI_TAG_XY_STD_DEV = 0.5;
    private static final double MEGATAG2_THETA_STD_DEV = 9999999.0;

    public LimelightCamera(String limelightName, Transform3d transform) {
        this.limelightName = limelightName;

        LimelightHelpers.setCameraPose_RobotSpace(
                limelightName,
                transform.getX(), // forward (+X)
                transform.getY(), // left (+Y)
                transform.getZ(), // up (+Z)
                Math.toDegrees(transform.getRotation().getX()), // roll
                Math.toDegrees(transform.getRotation().getY()), // pitch
                Math.toDegrees(transform.getRotation().getZ())  // yaw
        );
    }

    @Override
    public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
        this.lastYawRateDegPerSec = yawRateDegPerSec;
        // MegaTag 2 recommends passing yaw with zeroed pitch/roll/rates; spin filtering is checked in Java.
        LimelightHelpers.SetRobotOrientation(limelightName, yawDegrees, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    @Override
    public Optional<VisionMeasurement> getEstimatedGlobalPose() {
        // Reject measurements when the robot is spinning rapidly to avoid motion-blur/latency errors.
        if (Math.abs(lastYawRateDegPerSec) > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return Optional.empty();
        }

        PoseEstimate est = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);

        if (est == null || !LimelightHelpers.validPoseEstimate(est)
                || est.tagCount <= 0 || est.avgTagDist <= 0) {
            return Optional.empty();
        }

        // Reject impossible poses outside the field perimeter.
        double x = est.pose.getX();
        double y = est.pose.getY();
        if (x < 0.0 || x > VisionConstants.FIELD_LENGTH_METERS
                || y < 0.0 || y > VisionConstants.FIELD_WIDTH_METERS) {
            return Optional.empty();
        }

        // Reject far single-tag estimates completely instead of passing them to the pose estimator.
        if (est.tagCount == 1 && est.avgTagDist > VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS) {
            return Optional.empty();
        }

        double baseXY = est.tagCount > 1 ? MULTI_TAG_XY_STD_DEV : SINGLE_TAG_XY_STD_DEV;
        double xy = baseXY * (1.0 + (est.avgTagDist * est.avgTagDist / 30.0));
        Matrix<N3, N1> stdDevs = VecBuilder.fill(xy, xy, MEGATAG2_THETA_STD_DEV);

        VisionMeasurement measurement = new VisionMeasurement(
                est.pose, // Pose2d in WPILib blue frame
                est.timestampSeconds, // already latency-corrected
                stdDevs,
                est.tagCount,
                est.avgTagDist);

        return Optional.of(measurement);
    }

    @Override
    public String getName() {
        return limelightName;
    }
}
