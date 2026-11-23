package frc.robot;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Constants {
    public static final class CANIdConstants {
        // CTRE Drivetrain Drive Motor CAN IDs
        public static final int FRONT_RIGHT_DRIVE_CAN_ID = 1;
        public static final int BACK_RIGHT_DRIVE_CAN_ID = 3;
        public static final int BACK_LEFT_DRIVE_CAN_ID = 5;
        public static final int FRONT_LEFT_DRIVE_CAN_ID = 7;

        // CTRE Drivetrain Steer Motor CAN IDs
        public static final int FRONT_RIGHT_STEERING_CAN_ID = 2;
        public static final int BACK_RIGHT_STEERING_CAN_ID = 4;
        public static final int BACK_LEFT_STEERING_CAN_ID = 6;
        public static final int FRONT_LEFT_STEERING_CAN_ID = 8;

        // CTRE Drivetrain CANcoder CAN IDs
        public static final int FRONT_RIGHT_ENCODER_CAN_ID = 9;
        public static final int BACK_RIGHT_ENCODER_CAN_ID = 10;
        public static final int BACK_LEFT_ENCODER_CAN_ID = 11;
        public static final int FRONT_LEFT_ENCODER_CAN_ID = 12;

        // CTRE Gyro CAN ID
        public static final int PIGEON_GYRO_CAN_ID = 21;

        // Example Launcher CAN IDs (REV Vortex shooter + CTRE Kraken loader + CTRE CANrange)
        public static final int LAUNCHER_SHOOTER_ID = 41;
        public static final int LAUNCHER_LOADER_ID = 42;
        public static final int LAUNCHER_CANRANGE_ID = 43;

        // Example Intake CAN IDs (two REV NEOs for slapdown deploy + one REV Vortex for roller)
        public static final int INTAKE_DEPLOY_LEADER_ID = 51;
        public static final int INTAKE_DEPLOY_FOLLOWER_ID = 52;
        public static final int INTAKE_ROLLER_ID = 53;
    }

    public static final class OperatorConstants {
        public static final double DRIVE_DEADBAND = 0.10;
        public static final double DPAD_SPEED_REGULATOR = 0.20;
    }

    public static final class AutonConstants {
        // PID constants for holonomic auto control (PathPlanner)
        // TODO: Tune these values on the real CTRE robot
        public static final PIDConstants TRANSLATION_PID = new PIDConstants(5.0, 0.0, 0.0);
        public static final PIDConstants ANGLE_PID = new PIDConstants(5.0, 0.0, 0.0);

        // Holonomic controller used by AutoBuilder
        public static final PathFollowingController AUTON_CONTROLLER = new PPHolonomicDriveController(
                TRANSLATION_PID,
                ANGLE_PID);
    }

    public static final class VisionConstants {
        public static final String LIMELIGHT_NAME = "limelight";

        // Field bounds and MegaTag 2 rejection thresholds
        // TODO: Update field dimensions once the 2027 field manual is released
        public static final double FIELD_LENGTH_METERS = 16.54;
        public static final double FIELD_WIDTH_METERS = 8.07;
        public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 720.0;
        public static final double SINGLE_TAG_MAX_DISTANCE_METERS = 4.0;

        // +X = forward, +Y = left, +Z = up (meters from robot center)
        private static final Translation3d FRONT_RIGHT_CAMERA_TRANSLATION = new Translation3d(
                Units.inchesToMeters(-8.211985), // X: forward
                Units.inchesToMeters(-0.102096), // Y: left
                Units.inchesToMeters(17.611942)  // Z: height
        );
        // Camera orientation relative to robot frame (roll, pitch, yaw in degrees)
        private static final Rotation3d FRONT_RIGHT_LIMELIGHT_ROTATION = new Rotation3d(
                Math.toRadians(0.0),  // roll
                Math.toRadians(25.0), // pitch (looking slightly up)
                Math.toRadians(0.0)   // yaw
        );
        public static final Transform3d FRONT_RIGHT_LIMELIGHT_TRANSFORM = new Transform3d(
                FRONT_RIGHT_CAMERA_TRANSLATION,
                FRONT_RIGHT_LIMELIGHT_ROTATION);

        // Placeholder camera transforms/names for additional 2027 cameras.
        // TODO: Uncomment and measure X/Y/Z and roll/pitch/yaw once cameras are mounted on the 2027 robot.
        // public static final Transform3d SYSTEMCORE_LEFT_CAMERA_TRANSFORM = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
        //         new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));
        // public static final Transform3d SYSTEMCORE_RIGHT_CAMERA_TRANSFORM = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
        //         new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));
        // public static final String PHOTON_CAMERA_NAME = "photon_cam";
        // public static final Transform3d PHOTON_CAMERA_TRANSFORM = new Transform3d(
        //         new Translation3d(Units.inchesToMeters(0.0), Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)),
        //         new Rotation3d(Math.toRadians(0.0), Math.toRadians(0.0), Math.toRadians(0.0)));
    }
}
