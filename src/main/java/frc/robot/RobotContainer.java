// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.CancelCommand;
import frc.robot.commands.drive.FaceAprilTagCommand;
import frc.robot.commands.intake.DeployIntakeCommand;
import frc.robot.commands.intake.RunIntakeRollerCommand;
import frc.robot.commands.intake.StowIntakeCommand;
import frc.robot.commands.launch.LaunchBallCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleIntakeSubsystem;
import frc.robot.subsystems.ExampleLaunchSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic
 * methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and
 * trigger mappings) should be declared here.
 */
public class RobotContainer {
    private final double MaxSpeed = DriveSubsystem.MAX_SPEED;
    private final double MaxAngularRate = DriveSubsystem.MAX_ANGULAR_RATE;

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * OperatorConstants.DRIVE_DEADBAND)
            .withRotationalDeadband(MaxAngularRate * OperatorConstants.DRIVE_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric dpadDrive = new SwerveRequest.RobotCentric()
            .withRotationalDeadband(MaxAngularRate * OperatorConstants.DRIVE_DEADBAND)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // CTRE template requests (commented out for reference)
    // private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    // private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverXbox = new CommandXboxController(0);

    public final DriveSubsystem driveSubsystem;
    // VisionSubsystem runs pose updates automatically in its periodic() method.
    @SuppressWarnings("unused")
    private final VisionSubsystem visionSubsystem;
    private final ExampleIntakeSubsystem exampleIntakeSubsystem;
    private final ExampleLaunchSubsystem exampleLaunchSubsystem;

    private final SendableChooser<Command> autoChooser;

    /**
     * The container for the robot. Contains subsystems, IO devices, and commands.
     */
    public RobotContainer() {
        // Initialize all subsystems for the robot here.
        // DriveSubsystem must be created first so PathPlanner AutoBuilder is configured before buildAutoChooser().
        driveSubsystem = DriveSubsystem.create();
        visionSubsystem = new VisionSubsystem(driveSubsystem);
        exampleIntakeSubsystem = new ExampleIntakeSubsystem();
        exampleLaunchSubsystem = new ExampleLaunchSubsystem();

        // Register PathPlanner named commands before building the auto chooser.
        registerNamedCommands();

        // Setup all controller button mappings.
        configureBindings();

        // Choosers show up on our dashboard and let us select which PathPlanner auto runs during matches.
        autoChooser = AutoBuilder.buildAutoChooser();

        // For choosers (or other info) to show up on the dashboard, they must be added to SmartDashboard.
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    /**
     * Adding commands as PathPlanner NamedCommands allows autonomous routines
     * built in the PathPlanner GUI to trigger mechanism actions along a path.
     * Give each NamedCommand its own fresh command instance (do not share instances with button bindings).
     */
    private void registerNamedCommands() {
        NamedCommands.registerCommand("deployIntake", new DeployIntakeCommand(exampleIntakeSubsystem));
        NamedCommands.registerCommand("stowIntake", new StowIntakeCommand(exampleIntakeSubsystem));
        NamedCommands.registerCommand("intakeRoller", new RunIntakeRollerCommand(exampleIntakeSubsystem));
        NamedCommands.registerCommand("launchBall", new LaunchBallCommand(exampleLaunchSubsystem));
        NamedCommands.registerCommand(
                "cancel",
                new CancelCommand(List.of(exampleIntakeSubsystem, exampleLaunchSubsystem)));
    }

    /**
     * Configure the trigger bindings for the robot.
     *
     * In this function, we initialize commands and map them to controller buttons.
     */
    private void configureBindings() {
        // Some subsystems have default commands that run whenever no other command is using them.
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        // D-pad nudges robot-centrically at 20% speed; otherwise uses CTRE field-centric driving.
        driveSubsystem.setDefaultCommand(
            driveSubsystem.applyRequest(() -> {
                int pov = driverXbox.getHID().getPOV();
                if (pov != -1) {
                    double angleRad = Math.toRadians(pov);
                    double nudgeSpeed = OperatorConstants.DPAD_SPEED_REGULATOR * MaxSpeed;
                    return dpadDrive
                        .withVelocityX(Math.cos(angleRad) * nudgeSpeed)
                        .withVelocityY(-Math.sin(angleRad) * nudgeSpeed)
                        .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate);
                }
                return drive
                    .withVelocityX(-driverXbox.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverXbox.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverXbox.getRightX() * MaxAngularRate); // Drive counterclockwise with negative X (left)
            })
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            driveSubsystem.applyRequest(() -> idle).ignoringDisable(true)
        );

        // Example intake bindings: X deploys slapdown arm, Y stows it, Left Trigger spins the roller while held.
        driverXbox.x().onTrue(new DeployIntakeCommand(exampleIntakeSubsystem));
        driverXbox.y().onTrue(new StowIntakeCommand(exampleIntakeSubsystem));
        driverXbox.leftTrigger().whileTrue(new RunIntakeRollerCommand(exampleIntakeSubsystem));

        // Example launch binding: hold Right Trigger to spin up shooter and fire whenever CANrange detects a ball.
        driverXbox.rightTrigger().whileTrue(new LaunchBallCommand(exampleLaunchSubsystem));

        // Tactile feedback: vibrate the driver's Xbox controller while the shooter is up to speed in teleop.
        new Trigger(exampleLaunchSubsystem::isAtTargetVelocity)
                .and(RobotModeTriggers.teleop())
                .whileTrue(
                        Commands.startEnd(
                                () -> driverXbox.setRumble(RumbleType.kBothRumble, 0.5),
                                () -> driverXbox.setRumble(RumbleType.kBothRumble, 0.0)));

        // While held, the robot will auto-orient to any visible AprilTag while allowing field-centric driving.
        driverXbox.start().whileTrue(new FaceAprilTagCommand(driveSubsystem, driverXbox));

        // All cancelable mechanism subsystems (other than drive) should be passed in to CancelCommand.
        driverXbox.leftBumper().onTrue(
                new CancelCommand(List.of(exampleIntakeSubsystem, exampleLaunchSubsystem)));

        // Instant commands are useful for simple one-time actions.
        // Manually re-zero the field-centric heading if the robot becomes misaligned.
        driverXbox.rightStick().and(driverXbox.leftStick())
                .onTrue(Commands.runOnce(driveSubsystem::zeroHeading));

        // Example auto-align binding (commented out until 2027 field target coordinates are defined):
        // driverXbox.b().whileTrue(
        //     driveSubsystem.driveToPoseCommand(new Pose2d(2.8, 4.2, Rotation2d.fromDegrees(0.0)))
        // );

        // CTRE template bindings (commented out for reference; adjust/uncomment as needed)
        // driverXbox.a().whileTrue(driveSubsystem.applyRequest(() -> brake));
        // driverXbox.b().whileTrue(driveSubsystem.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-driverXbox.getLeftY(), -driverXbox.getLeftX()))
        // ));
        //
        // // Run SysId routines when holding back/start and X/Y.
        // // Note that each routine should be run exactly once in a single log.
        // driverXbox.back().and(driverXbox.y()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kForward));
        // driverXbox.back().and(driverXbox.x()).whileTrue(driveSubsystem.sysIdDynamic(Direction.kReverse));
        // driverXbox.start().and(driverXbox.y()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kForward));
        // driverXbox.start().and(driverXbox.x()).whileTrue(driveSubsystem.sysIdQuasistatic(Direction.kReverse));
        //
        // // Reset the field-centric heading on left bumper press.
        // driverXbox.leftBumper().onTrue(driveSubsystem.runOnce(driveSubsystem::seedFieldCentric));

        driveSubsystem.registerTelemetry(logger::telemeterize);
    }

    /**
     * Return the auto selected on the dashboard.
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();

        // // CTRE template simple drive forward auton (commented out for reference)
        // final var idle = new SwerveRequest.Idle();
        // return Commands.sequence(
        //     // Reset our field centric heading to match the robot
        //     // facing away from our alliance station wall (0 deg).
        //     driveSubsystem.runOnce(() -> driveSubsystem.seedFieldCentric(Rotation2d.kZero)),
        //     // Then slowly drive forward (away from us) for 5 seconds.
        //     driveSubsystem.applyRequest(() ->
        //         drive.withVelocityX(0.5)
        //             .withVelocityY(0)
        //             .withRotationalRate(0)
        //     )
        //     .withTimeout(5.0),
        //     // Finally idle for the rest of auton
        //     driveSubsystem.applyRequest(() -> idle)
        // );
    }
}
