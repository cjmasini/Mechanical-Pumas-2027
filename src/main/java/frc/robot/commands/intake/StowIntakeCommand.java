package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleIntakeSubsystem;

/**
 * Command that raises the slapdown intake back to its stowed position using the
 * SparkMax onboard PID controller, then finishes once the arm reaches target.
 */
public class StowIntakeCommand extends Command {

    private final ExampleIntakeSubsystem intakeSubsystem;

    /**
     * Creates a new StowIntakeCommand.
     *
     * @param intakeSubsystem The example intake subsystem to control
     */
    public StowIntakeCommand(ExampleIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        // addRequirements prevents another intake command from interrupting mid-motion unless allowed.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Keep commanding the stowed position setpoint every 20 ms loop.
        intakeSubsystem.stowIntake();
    }

    @Override
    public boolean isFinished() {
        // Finish automatically once the arm is within tolerance of the stowed setpoint.
        return intakeSubsystem.isStowed();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop driving the deploy motors once finished or interrupted; brake mode holds the arm upright.
        intakeSubsystem.stopDeploy();
    }
}
