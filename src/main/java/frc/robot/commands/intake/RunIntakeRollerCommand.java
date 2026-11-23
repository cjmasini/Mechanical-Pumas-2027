package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ExampleIntakeSubsystem;

/**
 * Command that spins the Vortex intake roller motor at a specified speed while scheduled
 * (for example, while a button/trigger is held, or until canceled).
 */
public class RunIntakeRollerCommand extends Command {

    private static final double DEFAULT_ROLLER_SPEED = 1.0;

    private final ExampleIntakeSubsystem intakeSubsystem;
    private final double speed;

    /**
     * Creates a new RunIntakeRollerCommand at full forward speed (1.0).
     *
     * @param intakeSubsystem The example intake subsystem to control
     */
    public RunIntakeRollerCommand(ExampleIntakeSubsystem intakeSubsystem) {
        this(intakeSubsystem, DEFAULT_ROLLER_SPEED);
    }

    /**
     * Creates a new RunIntakeRollerCommand at a custom roller speed.
     *
     * @param intakeSubsystem The example intake subsystem to control
     * @param speed           Roller speed from -1.0 (eject) to 1.0 (intake)
     */
    public RunIntakeRollerCommand(ExampleIntakeSubsystem intakeSubsystem, double speed) {
        this.intakeSubsystem = intakeSubsystem;
        this.speed = speed;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        // Start spinning the roller motor as soon as the command is scheduled.
        intakeSubsystem.setRollerSpeed(speed);
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        // Returning false means this command runs until the button is released (whileTrue)
        // or until CancelCommand / another intake command stops it.
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the roller when the command ends so it does not keep spinning unexpectedly.
        intakeSubsystem.stopRoller();
    }
}
