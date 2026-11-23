package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleIntakeSubsystem;

/**
 * Command that lowers the slapdown intake to its deployed position using the
 * SparkMax onboard PID controller, then finishes once the arm reaches target.
 */
public class DeployIntakeCommand extends Command {

    private final ExampleIntakeSubsystem intakeSubsystem;

    /**
     * Creates a new DeployIntakeCommand.
     *
     * @param intakeSubsystem The example intake subsystem to control
     */
    public DeployIntakeCommand(ExampleIntakeSubsystem intakeSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        // addRequirements ensures no two commands fight over the same subsystem at the same time.
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        // Keep commanding the deploy position setpoint every 20 ms loop.
        intakeSubsystem.deployIntake();
    }

    @Override
    public boolean isFinished() {
        // Finish automatically once the arm is within tolerance of the deployed setpoint.
        return intakeSubsystem.isDeployed();
    }

    @Override
    public void end(boolean interrupted) {
        // Stop driving the deploy motors once finished or interrupted; brake mode will hold the arm.
        intakeSubsystem.stopDeploy();
    }
}
