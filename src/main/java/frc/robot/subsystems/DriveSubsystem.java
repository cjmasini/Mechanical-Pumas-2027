package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutonConstants;
import frc.robot.commands.drive.DriveToPoseCommand;
import frc.robot.generated.TunerConstants;

/**
 * Extends CTRE's {@link CommandSwerveDrivetrain} with PathPlanner AutoBuilder
 * configuration and helper methods used by robot commands and vision.
 */
public class DriveSubsystem extends CommandSwerveDrivetrain {

    public static final double MAX_SPEED = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric fieldCentricRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentric fieldCentricBlueRequest = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
            .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance);

    private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
            .withDriveRequestType(DriveRequestType.Velocity);

    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        configureAutoBuilder();
    }

    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        configureAutoBuilder();
    }

    public DriveSubsystem(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        configureAutoBuilder();
    }

    /**
     * Creates a DriveSubsystem using the generated {@link TunerConstants}.
     */
    public static DriveSubsystem create() {
        return new DriveSubsystem(
                TunerConstants.DrivetrainConstants,
                TunerConstants.FrontLeft,
                TunerConstants.FrontRight,
                TunerConstants.BackLeft,
                TunerConstants.BackRight);
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,
                    (speeds, feedforwards) -> setControl(
                            pathApplyRobotSpeeds
                                    .withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    AutonConstants.AUTON_CONTROLLER,
                    config,
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this);
        } catch (Exception e) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
    }

    /**
     * Returns the currently-estimated pose of the robot.
     */
    public Pose2d getPose() {
        return getState().Pose;
    }

    /**
     * Gets current speeds of the robot in robot-relative coordinates.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return getState().Speeds;
    }

    /**
     * Re-seeds the field-centric forward direction to the robot's current heading.
     */
    public void zeroHeading() {
        seedFieldCentric();
    }

    /**
     * Drives the robot in operator-perspective field-centric mode.
     *
     * @param velocityX      Forward velocity (m/s)
     * @param velocityY      Leftward velocity (m/s)
     * @param rotationalRate Counterclockwise angular rate (rad/s)
     */
    public void driveFieldCentric(double velocityX, double velocityY, double rotationalRate) {
        setControl(
                fieldCentricRequest
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotationalRate));
    }

    /**
     * Drives the robot in Blue-alliance field-centric coordinates (used by pose-based commands
     * so field coordinates stay consistent on both alliances).
     *
     * @param velocityX      Field X velocity toward Red alliance wall (m/s)
     * @param velocityY      Field Y velocity toward left side of field from Blue perspective (m/s)
     * @param rotationalRate Counterclockwise angular rate (rad/s)
     */
    public void driveBlueFieldCentric(double velocityX, double velocityY, double rotationalRate) {
        setControl(
                fieldCentricBlueRequest
                        .withVelocityX(velocityX)
                        .withVelocityY(velocityY)
                        .withRotationalRate(rotationalRate));
    }

    /**
     * Creates a command that smoothly drives the robot to a target field pose.
     *
     * @param targetPose Desired target pose in WPILib Blue-alliance field coordinates
     * @return Command that drives to the target pose and finishes once aligned
     */
    public Command driveToPoseCommand(Pose2d targetPose) {
        return new DriveToPoseCommand(this, targetPose);
    }
}
