package frc.robot.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants.VisionConstants;

/**
 * VisionCamera implementation backed by PhotonVision (for example, a ThriftyCam
 * or USB camera running on an Orange Pi 5 coprocessor).
 *
 * Uses PhotonLib's multi-tag coprocessor SolvePNP first, falling back to gyro-constrained
 * distance-trig solve and finally lowest-ambiguity single-tag estimation, while draining
 * all unread frames each loop and rejecting bad measurements.
 */
public class PhotonVisionCamera implements VisionCamera {

    // When using gyro-constrained trig solve, trust the Pigeon 2 for heading (huge theta std dev).
    private static final double SINGLE_TAG_TRIG_XY_STD_DEV = 0.7;
    private static final double TRIG_SOLVE_THETA_STD_DEV = 9999999.0;

    private static final Matrix<N3, N1> SINGLE_TAG_AMBIGUITY_STD_DEVS = VecBuilder.fill(4.0, 4.0, 8.0);
    private static final Matrix<N3, N1> MULTI_TAG_STD_DEVS = VecBuilder.fill(0.5, 0.5, 1.0);
    private static final double MAX_SINGLE_TAG_AMBIGUITY = 0.2;

    private final String cameraName;
    private final PhotonCamera camera;
    private final PhotonPoseEstimator photonEstimator;
    private final AprilTagFieldLayout fieldLayout;

    private double lastYawRateDegPerSec = 0.0;

    /**
     * Creates a PhotonVisionCamera using WPILib's default field AprilTag layout.
     *
     * @param cameraName    Camera name configured in the PhotonVision web UI
     * @param robotToCamera Physical 3D transform from robot center to camera lens
     */
    public PhotonVisionCamera(String cameraName, Transform3d robotToCamera) {
        this(cameraName, robotToCamera, AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));
    }

    /**
     * Creates a PhotonVisionCamera with an explicit AprilTag field layout.
     *
     * @param cameraName    Camera name configured in the PhotonVision web UI
     * @param robotToCamera Physical 3D transform from robot center to camera lens
     * @param fieldLayout   AprilTag field layout for the current season
     */
    public PhotonVisionCamera(
            String cameraName,
            Transform3d robotToCamera,
            AprilTagFieldLayout fieldLayout) {
        this.cameraName = cameraName;
        this.fieldLayout = fieldLayout;
        this.camera = new PhotonCamera(cameraName);
        this.photonEstimator = new PhotonPoseEstimator(fieldLayout, robotToCamera);
    }

    @Override
    public void setRobotOrientation(double yawDegrees, double yawRateDegPerSec) {
        this.lastYawRateDegPerSec = yawRateDegPerSec;
        // Feed gyro heading so trig-solve / heading-constrained strategies stay synchronized.
        photonEstimator.addHeadingData(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(yawDegrees));
    }

    @Override
    public List<VisionMeasurement> getEstimatedGlobalPoses() {
        // Always drain unread results every loop so stale frames never pile up when spinning fast.
        List<PhotonPipelineResult> results = camera.getAllUnreadResults();

        if (Math.abs(lastYawRateDegPerSec) > VisionConstants.MAX_ANGULAR_VELOCITY_DEG_PER_SEC) {
            return List.of();
        }

        List<VisionMeasurement> measurements = new ArrayList<>();

        for (PhotonPipelineResult result : results) {
            boolean usedGyroHeadingFallback = false;
            boolean usedLowestAmbiguityFallback = false;

            // 1. Prefer multi-tag SolvePNP computed on the coprocessor (e.g. Orange Pi 5).
            Optional<EstimatedRobotPose> estOpt = photonEstimator.estimateCoprocMultiTagPose(result);

            // 2. Fallback to gyro-constrained distance-trig solve for single tags.
            if (estOpt.isEmpty()) {
                estOpt = photonEstimator.estimatePnpDistanceTrigSolvePose(result);
                if (estOpt.isPresent()) {
                    usedGyroHeadingFallback = true;
                }
            }

            // 3. Final fallback to lowest-ambiguity single-tag pose.
            if (estOpt.isEmpty()) {
                estOpt = photonEstimator.estimateLowestAmbiguityPose(result);
                if (estOpt.isPresent()) {
                    usedLowestAmbiguityFallback = true;
                }
            }

            if (estOpt.isEmpty()) {
                continue;
            }

            EstimatedRobotPose est = estOpt.get();
            Pose2d pose2d = est.estimatedPose.toPose2d();

            // Reject impossible poses outside the field perimeter.
            double x = pose2d.getX();
            double y = pose2d.getY();
            if (x < 0.0 || x > VisionConstants.FIELD_LENGTH_METERS
                    || y < 0.0 || y > VisionConstants.FIELD_WIDTH_METERS) {
                continue;
            }

            int tagCount = 0;
            double avgDist = 0.0;
            double singleTagAmbiguity = -1.0;

            for (PhotonTrackedTarget tgt : est.targetsUsed) {
                var tagPose = fieldLayout.getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                if (tagCount == 0) {
                    singleTagAmbiguity = tgt.getPoseAmbiguity();
                }
                tagCount++;
                avgDist += tagPose.get().toPose2d().getTranslation().getDistance(pose2d.getTranslation());
            }

            if (tagCount == 0) {
                continue;
            }

            avgDist /= tagCount;
            if (avgDist <= 0.0) {
                continue;
            }

            // Reject far single-tag estimates.
            if (tagCount == 1 && avgDist > VisionConstants.SINGLE_TAG_MAX_DISTANCE_METERS) {
                continue;
            }

            // Reject high-ambiguity single-tag estimates when gyro-constrained trig solve wasn't used.
            if (usedLowestAmbiguityFallback && tagCount == 1
                    && (singleTagAmbiguity < 0.0 || singleTagAmbiguity > MAX_SINGLE_TAG_AMBIGUITY)) {
                continue;
            }

            double scale = 1.0 + (avgDist * avgDist / 30.0);
            Matrix<N3, N1> stdDevs;
            if (tagCount > 1) {
                stdDevs = MULTI_TAG_STD_DEVS.times(scale);
            } else if (usedGyroHeadingFallback) {
                double xy = SINGLE_TAG_TRIG_XY_STD_DEV * scale;
                stdDevs = VecBuilder.fill(xy, xy, TRIG_SOLVE_THETA_STD_DEV);
            } else {
                stdDevs = SINGLE_TAG_AMBIGUITY_STD_DEVS.times(scale);
            }

            measurements.add(new VisionMeasurement(
                    pose2d,
                    est.timestampSeconds,
                    stdDevs,
                    tagCount,
                    avgDist));
        }

        return measurements;
    }

    @Override
    public Optional<VisionMeasurement> getEstimatedGlobalPose() {
        List<VisionMeasurement> poses = getEstimatedGlobalPoses();
        return poses.isEmpty() ? Optional.empty() : Optional.of(poses.get(poses.size() - 1));
    }

    @Override
    public String getName() {
        return cameraName;
    }
}
