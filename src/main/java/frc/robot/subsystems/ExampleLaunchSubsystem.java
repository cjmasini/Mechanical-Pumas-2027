package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CANIdConstants;

/**
 * ExampleLaunchSubsystem - Demonstrates combining REV and CTRE hardware in a single subsystem:
 * - A REV NEO Vortex shooter motor (on SparkFlex) controlled by closed-loop velocity PID + feedforward
 * - A CTRE Kraken X60 loader motor (on TalonFX) that feeds a ball into the shooter
 * - A CTRE CANrange Time-of-Flight (ToF) sensor that detects whether a ball is ready to launch
 *
 * Extending {@link CancelableSubsystemBase} ensures this subsystem implements {@link #cancel()},
 * allowing {@link frc.robot.commands.CancelCommand} to stop both motors safely.
 */
public class ExampleLaunchSubsystem extends CancelableSubsystemBase {

    // Example velocity PID and feedforward gains for the Vortex flywheel.
    // TODO: Tune these values on your actual launcher flywheel.
    private static final double SHOOTER_P = 0.0005;
    private static final double SHOOTER_I = 0.0;
    private static final double SHOOTER_D = 0.00005;
    // Feedforward (kV) estimates how much motor output is needed per RPM; PID only corrects the remaining error.
    private static final double SHOOTER_KV = 0.0018;

    private static final int SHOOTER_CURRENT_LIMIT = 40;
    private static final double VELOCITY_TOLERANCE_RPM = 50.0;

    // CANrange detection threshold and hysteresis in meters.
    // Threshold: distance closer than 0.15 m counts as a detected ball.
    // Hysteresis: prevents flickering between detected/not-detected right at the threshold edge.
    private static final double BALL_DETECT_THRESHOLD_METERS = 0.15;
    private static final double BALL_DETECT_HYSTERESIS_METERS = 0.02;

    /**
     * SparkFlex motor controller for the REV NEO Vortex shooter flywheel.
     */
    private final SparkFlex shooterMotor;
    private final RelativeEncoder shooterEncoder;
    private final SparkClosedLoopController shooterPID;

    /**
     * TalonFX motor controller for the CTRE Kraken loader motor.
     */
    private final TalonFX loaderMotor;

    /**
     * Reusable Phoenix 6 control request for the Kraken loader.
     * Reusing a single request object avoids creating extra garbage every 20 ms loop.
     */
    private final DutyCycleOut loaderRequest = new DutyCycleOut(0.0);

    /**
     * CTRE CANrange Time-of-Flight sensor used to detect when a ball is staged at the loader.
     */
    private final CANrange ballSensor;

    private double targetVelocityRPM = 0.0;

    /**
     * Initialize the Example Launch Subsystem.
     */
    public ExampleLaunchSubsystem() {
        this.setName("ExampleLaunchSubsystem");

        // --- 1. Configure REV NEO Vortex Shooter Motor (SparkFlex) ---
        this.shooterMotor = new SparkFlex(
                CANIdConstants.LAUNCHER_SHOOTER_ID,
                MotorType.kBrushless);

        SparkFlexConfig shooterConfig = new SparkFlexConfig();

        // Coast mode is recommended for flywheels so momentum bleeds off naturally instead of
        // slamming the mechanism when stopped.
        shooterConfig.idleMode(IdleMode.kCoast);
        shooterConfig.inverted(false);
        // 40 Amps and 12 volts are safe starting settings for most mechanisms.
        shooterConfig.smartCurrentLimit(SHOOTER_CURRENT_LIMIT);
        shooterConfig.voltageCompensation(12.0);

        // Configure onboard velocity closed-loop control (PID + kV feedforward) in RPM.
        shooterConfig.closedLoop
                .pid(SHOOTER_P, SHOOTER_I, SHOOTER_D)
                .outputRange(-1.0, 1.0)
                .feedForward.kV(SHOOTER_KV);

        // ResetMode.kResetSafeParameters resets unsafe params; PersistMode.kNoPersistParameters keeps changes in RAM.
        shooterMotor.configure(
                shooterConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kNoPersistParameters);

        this.shooterEncoder = shooterMotor.getEncoder();
        this.shooterPID = shooterMotor.getClosedLoopController();

        // --- 2. Configure CTRE Kraken Loader Motor (TalonFX) ---
        this.loaderMotor = new TalonFX(CANIdConstants.LAUNCHER_LOADER_ID);

        TalonFXConfiguration loaderConfig = new TalonFXConfiguration();
        // Brake mode keeps balls from slipping past the loader before the shooter is ready.
        loaderConfig.MotorOutput
                .withNeutralMode(NeutralModeValue.Brake)
                .withInverted(InvertedValue.CounterClockwise_Positive);
        // Supply current limits protect the battery/breaker; stator limits cap motor torque/heat.
        loaderConfig.CurrentLimits
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true);

        loaderMotor.getConfigurator().apply(loaderConfig);

        // --- 3. Configure CTRE CANrange Ball Sensor ---
        this.ballSensor = new CANrange(CANIdConstants.LAUNCHER_CANRANGE_ID);

        CANrangeConfiguration sensorConfig = new CANrangeConfiguration();
        // Configure the onboard ToF proximity detector so getIsDetected() reports whether a ball is present.
        sensorConfig.ProximityParams
                .withProximityThreshold(BALL_DETECT_THRESHOLD_METERS)
                .withProximityHysteresis(BALL_DETECT_HYSTERESIS_METERS);

        ballSensor.getConfigurator().apply(sensorConfig);
    }

    @Override
    public void periodic() {
        // Publish live shooter speed and CANrange detection state to SmartDashboard for tuning.
        SmartDashboard.putNumber("ExampleLaunch/ShooterRPM", getShooterVelocity());
        SmartDashboard.putNumber("ExampleLaunch/TargetRPM", targetVelocityRPM);
        SmartDashboard.putBoolean("ExampleLaunch/AtTargetVelocity", isAtTargetVelocity());
        SmartDashboard.putBoolean("ExampleLaunch/BallDetected", isBallDetected());
        SmartDashboard.putNumber("ExampleLaunch/SensorDistanceMeters", getSensorDistanceMeters());
    }

    /**
     * Command the Vortex shooter flywheel to spin up to a target velocity using onboard closed-loop control.
     *
     * @param targetRPM Desired flywheel velocity in RPM
     */
    public void setShooterVelocity(double targetRPM) {
        this.targetVelocityRPM = targetRPM;
        shooterPID.setSetpoint(targetRPM, ControlType.kVelocity);
    }

    /**
     * Get the current shooter flywheel velocity from the SparkFlex encoder.
     *
     * @return Current velocity in RPM
     */
    public double getShooterVelocity() {
        return shooterEncoder.getVelocity();
    }

    /**
     * Get the currently commanded target velocity.
     *
     * @return Target velocity in RPM
     */
    public double getTargetVelocity() {
        return targetVelocityRPM;
    }

    /**
     * Check whether the shooter flywheel has spun up close enough to its target velocity to fire.
     * Also checks {@code targetVelocityRPM > 0.0} so an idle shooter (0 == 0) does not look "ready".
     *
     * @return true if a positive target velocity is set and actual RPM is within tolerance
     */
    public boolean isAtTargetVelocity() {
        return targetVelocityRPM > 0.0
                && Math.abs(getShooterVelocity() - targetVelocityRPM) < VELOCITY_TOLERANCE_RPM;
    }

    /**
     * Stop the shooter flywheel (coasts down cleanly in {@link IdleMode#kCoast}).
     */
    public void stopShooter() {
        this.targetVelocityRPM = 0.0;
        this.shooterMotor.set(0);
    }

    /**
     * Run the Kraken loader motor at a duty-cycle speed (-1.0 to 1.0).
     *
     * @param speed Percent output (-1.0 to 1.0) to feed a ball into the shooter
     */
    public void setLoaderSpeed(double speed) {
        loaderMotor.setControl(loaderRequest.withOutput(speed));
    }

    /**
     * Stop the Kraken loader motor.
     */
    public void stopLoader() {
        loaderMotor.setControl(loaderRequest.withOutput(0.0));
    }

    /**
     * Check whether the CTRE CANrange sensor currently detects a ball within the configured threshold.
     *
     * @return true if a ball is detected
     */
    public boolean isBallDetected() {
        return Boolean.TRUE.equals(ballSensor.getIsDetected().getValue());
    }

    /**
     * Get the raw distance measured by the CTRE CANrange sensor.
     *
     * @return Measured distance in meters
     */
    public double getSensorDistanceMeters() {
        return ballSensor.getDistance().getValueAsDouble();
    }

    /**
     * Cancel function to safely stop both the shooter flywheel and the loader motor.
     */
    @Override
    public void cancel() {
        stopShooter();
        stopLoader();
    }
}
