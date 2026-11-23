package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

/**
 * Reusable command that drives the robot to a target {@link Pose2d} on the field
 * using smooth trapezoidal motion profiles on X, Y, and rotation.
 *
 * Useful for one-button auto-alignment to scoring locations, human player stations,
 * or climbing spots once vision pose estimation is running.
 */
public class DriveToPoseCommand extends Command {

    // TODO: Tune motion constraints and PID gains on the real robot.
    private static final double MAX_TRANSLATION_SPEED = 3.0; // m/s
    private static final double MAX_TRANSLATION_ACCEL = 3.0; // m/s^2
    private static final double MAX_ROTATION_SPEED = Math.PI; // rad/s
    private static final double MAX_ROTATION_ACCEL = 2.0 * Math.PI; // rad/s^2

    private static final double TRANSLATION_KP = 3.0;
    private static final double ROTATION_KP = 4.0;

    private static final double POSITION_TOLERANCE_METERS = 0.03;
    private static final double ROTATION_TOLERANCE_RADIANS = Math.toRadians(2.0);

    private final DriveSubsystem driveSubsystem;
    private final Pose2d targetPose;

    // ProfiledPIDControllers limit acceleration and max speed so the robot approaches smoothly without slamming.
    private final ProfiledPIDController xController = new ProfiledPIDController(
            TRANSLATION_KP, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_TRANSLATION_SPEED, MAX_TRANSLATION_ACCEL));
    private final ProfiledPIDController yController = new ProfiledPIDController(
            TRANSLATION_KP, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_TRANSLATION_SPEED, MAX_TRANSLATION_ACCEL));
    private final ProfiledPIDController thetaController = new ProfiledPIDController(
            ROTATION_KP, 0.0, 0.0,
            new TrapezoidProfile.Constraints(MAX_ROTATION_SPEED, MAX_ROTATION_ACCEL));

    /**
     * Creates a new DriveToPoseCommand.
     *
     * @param driveSubsystem The drive subsystem to control
     * @param targetPose     Target field pose in WPILib Blue-alliance coordinates
     */
    public DriveToPoseCommand(DriveSubsystem driveSubsystem, Pose2d targetPose) {
        this.driveSubsystem = driveSubsystem;
        this.targetPose = targetPose;

        // Continuous input allows rotation to wrap cleanly across -180/+180 degrees via the shortest path.
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        xController.setTolerance(POSITION_TOLERANCE_METERS);
        yController.setTolerance(POSITION_TOLERANCE_METERS);
        thetaController.setTolerance(ROTATION_TOLERANCE_RADIANS);

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        // Reset controllers to the robot's current pose so the motion profile starts smoothly from where we are.
        Pose2d currentPose = driveSubsystem.getPose();
        xController.reset(currentPose.getX());
        yController.reset(currentPose.getY());
        thetaController.reset(currentPose.getRotation().getRadians());

        xController.setGoal(targetPose.getX());
        yController.setGoal(targetPose.getY());
        thetaController.setGoal(targetPose.getRotation().getRadians());
    }

    @Override
    public void execute() {
        Pose2d currentPose = driveSubsystem.getPose();

        double vx = xController.atGoal() ? 0.0 : MathUtil.clamp(
                xController.calculate(currentPose.getX()),
                -MAX_TRANSLATION_SPEED, MAX_TRANSLATION_SPEED);
        double vy = yController.atGoal() ? 0.0 : MathUtil.clamp(
                yController.calculate(currentPose.getY()),
                -MAX_TRANSLATION_SPEED, MAX_TRANSLATION_SPEED);
        double omega = thetaController.atGoal() ? 0.0 : MathUtil.clamp(
                thetaController.calculate(currentPose.getRotation().getRadians()),
                -MAX_ROTATION_SPEED, MAX_ROTATION_SPEED);

        // Drive in Blue-alliance field coordinates so Pose2d targets behave identically on both alliances.
        driveSubsystem.driveBlueFieldCentric(vx, vy, omega);
    }

    @Override
    public boolean isFinished() {
        return xController.atGoal() && yController.atGoal() && thetaController.atGoal();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop moving once we reach the target pose or get interrupted.
        driveSubsystem.driveBlueFieldCentric(0.0, 0.0, 0.0);
    }
}
