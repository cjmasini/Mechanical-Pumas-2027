package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.vision.LimelightCamera;
// import frc.robot.vision.PhotonVisionCamera;
// import frc.robot.vision.SystemcoreThriftyCamera;
// import frc.robot.vision.SystemcoreThriftyCamera.SystemcoreUsbPort;
import frc.robot.vision.VisionCamera;
import frc.robot.vision.VisionMeasurement;

/**
 * VisionSubsystem manages any combination of {@link VisionCamera}s
 * (Limelight, Systemcore-hosted ThriftyCam, and PhotonVision)
 * and feeds their filtered pose estimates into {@link DriveSubsystem}.
 */
public class VisionSubsystem extends SubsystemBase {

    private final DriveSubsystem driveSubsystem;

    // All vision cameras
    private final VisionCamera[] cameras;

    // Track when we last had a valid vision measurement
    private double lastVisionTimestamp = 0.0;
    private static final double VISION_STALE_THRESHOLD_SECONDS = 0.5;

    /**
     * Constructs a VisionSubsystem that manages all vision cameras.
     *
     * @param driveSubsystem The drivetrain subsystem
     */
    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        this.cameras = new VisionCamera[] {
                new LimelightCamera(
                        VisionConstants.LIMELIGHT_NAME,
                        VisionConstants.FRONT_RIGHT_LIMELIGHT_TRANSFORM)
                // Uncomment cameras below (and their transforms in VisionConstants) once mounted:
                // , new SystemcoreThriftyCamera(
                //         SystemcoreUsbPort.USB0,
                //         VisionConstants.SYSTEMCORE_LEFT_CAMERA_TRANSFORM)
                // , new SystemcoreThriftyCamera(
                //         SystemcoreUsbPort.USB1,
                //         VisionConstants.SYSTEMCORE_RIGHT_CAMERA_TRANSFORM)
                // , new PhotonVisionCamera(
                //         VisionConstants.PHOTON_CAMERA_NAME,
                //         VisionConstants.PHOTON_CAMERA_TRANSFORM)
        };
    }

    @Override
    public void periodic() {
        // Feed current robot yaw (WPILib Blue-alliance frame) and angular rate into all cameras.
        double robotYawDeg = driveSubsystem.getPose().getRotation().getDegrees();
        double yawRateDegPerSec = driveSubsystem.getPigeon2().getAngularVelocityZWorld().getValueAsDouble();

        for (VisionCamera camera : cameras) {
            camera.setRobotOrientation(robotYawDeg, yawRateDegPerSec);

            for (VisionMeasurement m : camera.getEstimatedGlobalPoses()) {
                double measurementAge = Timer.getFPGATimestamp() - m.timestampSeconds;
                // Skip old measurements
                if (measurementAge > VISION_STALE_THRESHOLD_SECONDS) {
                    continue;
                }
                logMeasurement(camera, m);

                driveSubsystem.addVisionMeasurement(m.pose, m.timestampSeconds, m.stdDevs);
                lastVisionTimestamp = Timer.getFPGATimestamp();
            }
        }

        // Publish vision status
        SmartDashboard.putBoolean("Vision/HasRecentVision", hasRecentVision());
        SmartDashboard.putNumber("Vision/SecondsSinceLastVision", getSecondsSinceLastVision());
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

    /**
     * Checks if we have received a vision measurement recently.
     *
     * @return true if vision data is fresh (within threshold)
     */
    public boolean hasRecentVision() {
        return getSecondsSinceLastVision() < VISION_STALE_THRESHOLD_SECONDS;
    }

    /**
     * Gets the time since the last valid vision measurement.
     *
     * @return Seconds since last vision update
     */
    public double getSecondsSinceLastVision() {
        if (lastVisionTimestamp == 0.0) {
            return Double.MAX_VALUE;
        }
        return Timer.getFPGATimestamp() - lastVisionTimestamp;
    }
}
