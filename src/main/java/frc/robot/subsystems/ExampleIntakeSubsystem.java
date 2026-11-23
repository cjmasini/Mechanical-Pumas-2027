package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;

/**
 * ExampleIntakeSubsystem - Demonstrates a slapdown intake mechanism using:
 * - Two NEO motors on SparkMax controllers in a leader/follower configuration to deploy and stow the intake arm
 * - One NEO Vortex motor on a SparkFlex controller to spin the intake roller
 *
 * Extending {@link CancelableSubsystemBase} ensures this subsystem implements {@link #cancel()},
 * allowing {@link frc.robot.commands.CancelCommand} to safely stop all mechanisms at once.
 */
public class ExampleIntakeSubsystem extends CancelableSubsystemBase {

    // Example deploy arm setpoints in motor rotations (0 = starting/stowed position).
    // TODO: Tune these positions and PID gains on your actual robot mechanism.
    private static final double STOWED_POSITION = 0.0;
    private static final double DEPLOYED_POSITION = -11.4;
    private static final double POSITION_TOLERANCE = 0.1;

    private static final double DEPLOY_P = 0.05;
    private static final double DEPLOY_I = 0.0;
    private static final double DEPLOY_D = 0.02;

    // Current limits protect motors from burning out if the mechanism stalls.
    private static final int DEPLOY_CURRENT_LIMIT = 40;
    private static final int ROLLER_CURRENT_LIMIT = 30;

    /**
     * Leader and follower SparkMax motor controllers for the slapdown NEO motors.
     * SparkMax is the standard controller for REV NEO (and NEO 550) motors.
     */
    private final SparkMax leaderDeployMotor;
    private final SparkMax followerDeployMotor;

    /**
     * SparkFlex motor controller for the intake roller.
     * SparkFlex is used with REV NEO Vortex motors.
     */
    private final SparkFlex rollerMotor;

    /**
     * Built-in relative encoders and closed-loop PID controller on the Spark controllers.
     */
    private final RelativeEncoder deployEncoder;
    private final SparkClosedLoopController deployPID;
    private final RelativeEncoder rollerEncoder;

    /**
     * Initialize the Example Intake Subsystem.
     */
    public ExampleIntakeSubsystem() {
        this.setName("ExampleIntakeSubsystem");

        // --- Slapdown Deploy Motors (2x REV NEO on SparkMax, leader/follower) ---
        this.leaderDeployMotor = new SparkMax(
                CANIdConstants.INTAKE_DEPLOY_LEADER_ID,
                MotorType.kBrushless);

        this.followerDeployMotor = new SparkMax(
                CANIdConstants.INTAKE_DEPLOY_FOLLOWER_ID,
                MotorType.kBrushless);

        SparkMaxConfig leaderDeployConfig = new SparkMaxConfig();
        SparkMaxConfig followerDeployConfig = new SparkMaxConfig();

        // Brake mode holds the slapdown arm in place against gravity when no power is applied.
        leaderDeployConfig.idleMode(IdleMode.kBrake);
        // 40 Amps and 12 volts are safe starting settings to prevent brownouts and stalling damage.
        leaderDeployConfig.smartCurrentLimit(DEPLOY_CURRENT_LIMIT);
        // Voltage compensation scales motor output relative to 12V so behavior stays consistent as the battery drains.
        leaderDeployConfig.voltageCompensation(12.0);
        // Configure the SparkMax's onboard PID controller for closed-loop position control.
        leaderDeployConfig.closedLoop
                .p(DEPLOY_P)
                .i(DEPLOY_I)
                .d(DEPLOY_D);

        followerDeployConfig.idleMode(IdleMode.kBrake);
        followerDeployConfig.smartCurrentLimit(DEPLOY_CURRENT_LIMIT);
        followerDeployConfig.voltageCompensation(12.0);
        // Mirror every command sent to the leader motor onto the follower motor.
        // Passing 'true' for inverted makes the follower spin in the opposite direction,
        // which is required when two motors face each other on opposite sides of the same shaft/mechanism.
        followerDeployConfig.follow(CANIdConstants.INTAKE_DEPLOY_LEADER_ID, true);

        // Configure both SparkMax controllers.
        // ResetMode.kResetSafeParameters clears old settings before applying new ones.
        // PersistMode.kNoPersistParameters applies settings in RAM only (fast startup, no flash wear).
        leaderDeployMotor.configure(
                leaderDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);
        followerDeployMotor.configure(
                followerDeployConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.deployEncoder = leaderDeployMotor.getEncoder();
        this.deployPID = leaderDeployMotor.getClosedLoopController();

        // --- Intake Roller Motor (1x REV NEO Vortex on SparkFlex) ---
        this.rollerMotor = new SparkFlex(
                CANIdConstants.INTAKE_ROLLER_ID,
                MotorType.kBrushless);

        SparkFlexConfig rollerConfig = new SparkFlexConfig();

        // Coast mode lets the roller spin down smoothly without abruptly gripping game pieces.
        // Switch to IdleMode.kBrake if you want the roller to stop immediately.
        rollerConfig.idleMode(IdleMode.kCoast);
        rollerConfig.inverted(true);
        rollerConfig.smartCurrentLimit(ROLLER_CURRENT_LIMIT);
        rollerConfig.voltageCompensation(12.0);

        rollerMotor.configure(
                rollerConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.rollerEncoder = rollerMotor.getEncoder();
    }

    @Override
    public void periodic() {
        // Publish live mechanism telemetry to SmartDashboard for tuning and debugging.
        SmartDashboard.putNumber("ExampleIntake/DeployPosition", deployEncoder.getPosition());
        SmartDashboard.putNumber("ExampleIntake/DeploySetpoint", deployPID.getSetpoint());
        SmartDashboard.putNumber("ExampleIntake/RollerRPM", rollerEncoder.getVelocity());
    }

    /**
     * Command the slapdown intake to move to its deployed position.
     * Call periodically until it returns true.
     *
     * @return true if the deploy arm has reached the deployed position
     */
    public boolean deployIntake() {
        return setIntakePosition(DEPLOYED_POSITION);
    }

    /**
     * Command the slapdown intake to move to its stowed position.
     * Call periodically until it returns true.
     *
     * @return true if the deploy arm has reached the stowed position
     */
    public boolean stowIntake() {
        return setIntakePosition(STOWED_POSITION);
    }

    /**
     * Command the deploy arm to a target position using the SparkMax onboard PID controller.
     * Because the follower tracks the leader automatically, we only command the leader.
     *
     * @param targetPosition Target position in motor rotations
     * @return true if within {@link #POSITION_TOLERANCE} of targetPosition
     */
    public boolean setIntakePosition(double targetPosition) {
        deployPID.setSetpoint(targetPosition, ControlType.kPosition);
        return Math.abs(targetPosition - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    /**
     * Check if the deploy arm is currently at the deployed position.
     *
     * @return true if deployed within tolerance
     */
    public boolean isDeployed() {
        return Math.abs(DEPLOYED_POSITION - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    /**
     * Check if the deploy arm is currently at the stowed position.
     *
     * @return true if stowed within tolerance
     */
    public boolean isStowed() {
        return Math.abs(STOWED_POSITION - deployEncoder.getPosition()) < POSITION_TOLERANCE;
    }

    /**
     * Get the current deploy arm position in motor rotations.
     *
     * @return Current encoder position
     */
    public double getIntakePosition() {
        return deployEncoder.getPosition();
    }

    /**
     * Set the intake roller motor speed.
     *
     * @param speed Speed (-1 to 1) to spin the intake roller
     */
    public void setRollerSpeed(double speed) {
        this.rollerMotor.set(speed);
    }

    /**
     * Stop the intake roller motor.
     */
    public void stopRoller() {
        this.rollerMotor.set(0);
    }

    /**
     * Stop the deploy motors (the follower stops automatically with the leader).
     */
    public void stopDeploy() {
        this.leaderDeployMotor.set(0);
    }

    /**
     * Cancel function to safely stop both the intake roller and deploy motors.
     */
    @Override
    public void cancel() {
        stopRoller();
        stopDeploy();
    }
}
