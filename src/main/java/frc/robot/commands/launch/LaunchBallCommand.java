package frc.robot.commands.launch;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleLaunchSubsystem;

/**
 * Example command that spins up the Vortex shooter motor to a target RPM and uses
 * the CTRE CANrange sensor to decide when to feed the ball with the Kraken loader:
 * 1. Immediately starts spinning the shooter flywheel up to {@code targetRPM}.
 * 2. Each loop, checks whether the flywheel is up to speed AND the CANrange detects a ball.
 * 3. Only runs the Kraken loader motor when both conditions are true; otherwise holds the loader stopped.
 */
public class LaunchBallCommand extends Command {

    private static final double DEFAULT_TARGET_RPM = 3250.0;
    private static final double LOADER_FEED_SPEED = 0.75;

    private final ExampleLaunchSubsystem launchSubsystem;
    private final double targetRPM;

    /**
     * Creates a new LaunchBallCommand using the default target velocity (3250 RPM).
     *
     * @param launchSubsystem The example launch subsystem to control
     */
    public LaunchBallCommand(ExampleLaunchSubsystem launchSubsystem) {
        this(launchSubsystem, DEFAULT_TARGET_RPM);
    }

    /**
     * Creates a new LaunchBallCommand with a custom target flywheel velocity.
     *
     * @param launchSubsystem The example launch subsystem to control
     * @param targetRPM       Desired Vortex shooter velocity in RPM
     */
    public LaunchBallCommand(ExampleLaunchSubsystem launchSubsystem, double targetRPM) {
        this.launchSubsystem = launchSubsystem;
        this.targetRPM = targetRPM;
        // addRequirements ensures no other command tries to control the launcher at the same time.
        addRequirements(launchSubsystem);
    }

    @Override
    public void initialize() {
        // Start spinning up the Vortex shooter flywheel right away.
        launchSubsystem.setShooterVelocity(targetRPM);
    }

    @Override
    public void execute() {
        // Keep commanding the target shooter RPM every loop.
        launchSubsystem.setShooterVelocity(targetRPM);

        // Only fire the ball into the shooter once the flywheel is at target speed AND a ball is detected.
        if (launchSubsystem.isAtTargetVelocity() && launchSubsystem.isBallDetected()) {
            launchSubsystem.setLoaderSpeed(LOADER_FEED_SPEED);
        } else {
            // Hold the ball back if the flywheel is still spinning up or no ball is in front of the CANrange.
            launchSubsystem.stopLoader();
        }
    }

    @Override
    public boolean isFinished() {
        // Runs continuously while held (whileTrue) or until CancelCommand stops it.
        // Tip: To make this finish automatically after firing one ball, you can track a boolean
        // flag (sawBall -> !isBallDetected()) and return true once the ball leaves the CANrange.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Safely stop both the shooter flywheel and the Kraken loader when the command ends.
        launchSubsystem.cancel();
    }
}
