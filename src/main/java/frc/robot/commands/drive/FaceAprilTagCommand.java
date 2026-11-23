package frc.robot.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.vision.LimelightHelpers;

/**
 * Command to drive the robot with normal linear field-centric joystick control while
 * automatically turning to face a detected AprilTag using Limelight {@code tx}.
 * Falls back to manual right-stick rotation when no AprilTag is visible.
 */
public class FaceAprilTagCommand extends Command {

    // Proportional gain converting horizontal angle error (radians) to rotational speed (rad/s).
    // TODO: Tune this gain on the real robot.
    private static final double APRILTAG_AIM_KP = 4.0;

    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController driverXbox;

    public FaceAprilTagCommand(DriveSubsystem driveSubsystem, CommandXboxController driverXbox) {
        this.driveSubsystem = driveSubsystem;
        this.driverXbox = driverXbox;
        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        // Match regular teleop driving: linear joystick inputs with the shared deadband.
        double vx = -MathUtil.applyDeadband(driverXbox.getLeftY(), OperatorConstants.DRIVE_DEADBAND)
                * DriveSubsystem.MAX_SPEED;
        double vy = -MathUtil.applyDeadband(driverXbox.getLeftX(), OperatorConstants.DRIVE_DEADBAND)
                * DriveSubsystem.MAX_SPEED;

        double omega;
        if (LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME)) {
            double txDeg = LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
            // Positive tx means the target is to the right, so rotate clockwise (negative omega).
            omega = MathUtil.clamp(
                    -Math.toRadians(txDeg) * APRILTAG_AIM_KP,
                    -DriveSubsystem.MAX_ANGULAR_RATE,
                    DriveSubsystem.MAX_ANGULAR_RATE);
        } else {
            omega = -MathUtil.applyDeadband(driverXbox.getRightX(), OperatorConstants.DRIVE_DEADBAND)
                    * DriveSubsystem.MAX_ANGULAR_RATE;
        }

        driveSubsystem.driveFieldCentric(vx, vy, omega);
    }
}
