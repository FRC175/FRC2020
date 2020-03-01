package com.team175.robot;

import com.team175.robot.commands.auto.EightBallAutoClose;
import com.team175.robot.commands.colorwheelspinner.SpinColorWheelToColor;
import com.team175.robot.commands.shooter.LockOntoTarget;
import com.team175.robot.commands.shooter.RotateTurretToFieldOrientedCardinal;
import com.team175.robot.models.AdvancedXboxController;
import com.team175.robot.models.XboxButton;
import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Climber;
import com.team175.robot.subsystems.ColorWheelSpinner;
import com.team175.robot.positions.TurretCardinal;
import com.team175.robot.subsystems.LED;
import com.team175.robot.subsystems.Drive;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import com.team175.robot.utils.ConnectionMonitor;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * RobotContainer is where the bulk of the robot should be declared.  Since Command-based is a "declarative" paradigm,
 * very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public final class RobotContainer {

    private final Drive drive;
    private final Limelight limelight;
    private final Shooter shooter;
    private final ColorWheelSpinner colorWheelSpinner;
    private final Intake intake;
    private final Climber climber;
    private final LED led;
    private final AdvancedXboxController driverController, operatorController;
    private final SendableChooser<Command> autoChooser;
    private final Logger logger;
    private final ConnectionMonitor monitor;

    private Command autoMode;
    private boolean isRobotPaused;

    private static RobotContainer instance;

    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int OPERATOR_CONTROLLER_PORT = 1;
    private static final double CONTROLLER_DEADBAND = 0.1;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        drive = Drive.getInstance();
        limelight = Limelight.getInstance();
        shooter = Shooter.getInstance();
        colorWheelSpinner = ColorWheelSpinner.getInstance();
        intake = Intake.getInstance();
        climber = Climber.getInstance();
        led = LED.getInstance();
        driverController = new AdvancedXboxController(DRIVER_CONTROLLER_PORT, CONTROLLER_DEADBAND);
        operatorController = new AdvancedXboxController(OPERATOR_CONTROLLER_PORT, CONTROLLER_DEADBAND);
        autoChooser = new SendableChooser<>();
        logger = LoggerFactory.getLogger(getClass().getSimpleName());
        monitor = ConnectionMonitor.getInstance();
        isRobotPaused = false;

        configureDefaultCommands();
        configureButtonBindings();
        configureAutoChooser();
        configureLogging();
    }

    public static RobotContainer getInstance() {
        if (instance == null) {
            instance = new RobotContainer();
        }

        return instance;
    }

    private void configureDefaultCommands() {
        // Arcade Drive
        drive.setDefaultCommand(
                new RunCommand(
                        () -> {
                            double throttle = driverController.getTriggerAxis(GenericHID.Hand.kRight) - driverController.getTriggerAxis(GenericHID.Hand.kLeft);
                            drive.arcadeDrive(
                                    throttle,
                                    driverController.getX(GenericHID.Hand.kLeft)
                            );
                            /*driverController.setRumble(GenericHID.RumbleType.kLeftRumble, Math.abs(throttle));
                            driverController.setRumble(GenericHID.RumbleType.kRightRumble, Math.abs(throttle));*/
                        },
                        drive
                ).andThen(() -> drive.arcadeDrive(0, 0), drive)
        );
    }

    private void configureButtonBindings() {
        // ----------------------------------------------------------------------------------------------------
        // DRIVE
        // ----------------------------------------------------------------------------------------------------
        // Shift gears
        new XboxButton(driverController, AdvancedXboxController.Button.X)
                .whileHeld(() -> drive.shift(true), drive)
                .whenReleased(() -> drive.shift(false), drive);

        // ----------------------------------------------------------------------------------------------------
        // INTAKE
        // ----------------------------------------------------------------------------------------------------
        // Intake control
        new XboxButton(driverController, AdvancedXboxController.Button.A)
                .whileHeld(() -> intake.setRollerOpenLoop(0.1), intake)
                .whenPressed(() -> intake.setIndexerOpenLoop(1), intake)
                .whenReleased(() -> intake.setRollerOpenLoop(0), intake);
        // Toggle indexer
        new XboxButton(driverController, AdvancedXboxController.Button.Y)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(() -> intake.setIndexerOpenLoop(1), intake),
                        new InstantCommand(() -> intake.setIndexerOpenLoop(0), intake),
                        () -> !intake.isIndexerMoving()
                ));
        // Reverse intake subsystem
        new XboxButton(driverController, AdvancedXboxController.Button.B)
                .whileHeld(
                        () -> {
                            intake.setIndexerOpenLoop(-1);
                            intake.setRollerOpenLoop(-0.1);
                        },
                        intake
                )
                .whenReleased(
                        () -> {
                            intake.setIndexerOpenLoop(0);
                            intake.setRollerOpenLoop(0);
                        },
                        intake
                );

        // ----------------------------------------------------------------------------------------------------
        // SHOOTER
        // ----------------------------------------------------------------------------------------------------
        // Manual Turret Control
        new XboxButton(operatorController, AdvancedXboxController.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(
                        () -> shooter.setTurretOpenLoop(-operatorController.getX(GenericHID.Hand.kRight)),
                        shooter
                ))
                .whenReleased(() -> shooter.setTurretOpenLoop(0), shooter);
        // Align to target
        new XboxButton(operatorController, AdvancedXboxController.Trigger.LEFT)
                .toggleWhenPressed(new LockOntoTarget(shooter, limelight));
                /*.whenReleased(new SequentialCommandGroup(
                        new InstantCommand(() -> {
                            operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 1);
                            operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 1);
                        }),
                        new WaitCommand(1),
                        new InstantCommand(() -> {
                            operatorController.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
                            operatorController.setRumble(GenericHID.RumbleType.kRightRumble, 0);
                        })
                ));*/
        // Auto shoot
        new XboxButton(operatorController, AdvancedXboxController.Trigger.RIGHT)
                // .whenPressed(new LogCommand("Pew pew"));
                .toggleWhenPressed(new FunctionalCommand(
                        () -> {
                            shooter.setBallGate(true);
                            shooter.setFlywheelOpenLoop(0.5);
                            shooter.setHoodHeading(Rotation2d.fromDegrees(90));
                        },
                        () -> {},
                        (interrupted) -> {
                            shooter.setBallGate(false);
                            shooter.setFlywheelOpenLoop(0);
                            shooter.setHoodHeading(Rotation2d.fromDegrees(0));
                        },
                        () -> false,
                        shooter
                ));
        // Turret Cardinals
        new XboxButton(operatorController, AdvancedXboxController.DPad.UP)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.NORTH));
        new XboxButton(operatorController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.EAST));
        new XboxButton(operatorController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.SOUTH));
        new XboxButton(operatorController, AdvancedXboxController.DPad.LEFT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.WEST));

        // ----------------------------------------------------------------------------------------------------
        // COLOR WHEEL SPINNER
        // ----------------------------------------------------------------------------------------------------
        // Deploy/retract color wheel spinner
        new XboxButton(operatorController, AdvancedXboxController.Button.A)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(colorWheelSpinner::deploy, colorWheelSpinner),
                        new InstantCommand(colorWheelSpinner::retract, colorWheelSpinner),
                        () -> !colorWheelSpinner.isDeployed()
                ));
        // Rotate wheel three times
        new XboxButton(operatorController, AdvancedXboxController.Button.X)
                .toggleWhenPressed(new FunctionalCommand(
                        colorWheelSpinner::spinWheel,
                        () -> {},
                        (interrupted) -> colorWheelSpinner.setOpenLoop(0),
                        colorWheelSpinner::hasSpunWheel,
                        colorWheelSpinner
                ));
        // Spin to color
        new XboxButton(operatorController, AdvancedXboxController.Button.B)
                .toggleWhenPressed(new SpinColorWheelToColor(colorWheelSpinner));

        // ----------------------------------------------------------------------------------------------------
        // CLIMBER
        // ----------------------------------------------------------------------------------------------------
        // Deploy/retract climber
        new XboxButton(driverController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(new ConditionalCommand(
                        new InstantCommand(climber::deploy, climber),
                        new InstantCommand(climber::retract, climber),
                        () -> !climber.isDeployed()
                ));
        // Winch up
        new XboxButton(driverController, AdvancedXboxController.DPad.UP)
                .whileHeld(() -> climber.setWinchOpenLoop(0.75), climber)
                .whenReleased(() -> climber.setWinchOpenLoop(0), climber);
        // Winch down
        new XboxButton(driverController, AdvancedXboxController.DPad.DOWN)
                .whileHeld(() -> climber.setWinchOpenLoop(-0.75), climber)
                .whenReleased(() -> climber.setWinchOpenLoop(0), climber);

        // ----------------------------------------------------------------------------------------------------
        // OLD
        // ----------------------------------------------------------------------------------------------------
        /*
        // Manual winch
        new XboxButton(driverController, AdvancedXboxController.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(
                        () -> climber.setWinchOpenLoop(driverController.getY(GenericHID.Hand.kRight)),
                        climber
                ))
                .whenReleased(() -> climber.setWinchOpenLoop(0), climber);

        // Deploy/retract ball gate
        new XboxButton(operatorController, AdvancedXboxController.DPad.UP)
                .whenPressed(() -> shooter.setBallGate(!shooter.getBallGate()), shooter);
        */
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", null);
        // Add more auto modes here
        autoChooser.addOption("Eight Ball Auto", new EightBallAutoClose(shooter, limelight, intake));
        SmartDashboard.putData("Auto Mode Chooser", autoChooser);
    }

    private void configureLogging() {
        io.github.oblarg.oblog.Logger.configureLoggingAndConfig(this, false);

        CommandScheduler.getInstance().onCommandInitialize(
                command -> {
                    Shuffleboard.addEventMarker("Command initialized.", command.getName(), EventImportance.kNormal);
                    logger.debug("{} initialized.", command.getName());
                }
        );
        CommandScheduler.getInstance().onCommandInterrupt(
                command -> {
                    Shuffleboard.addEventMarker("Command interrupted.", command.getName(), EventImportance.kNormal);
                    logger.debug("{} interrupted.", command.getName());
                }
        );
        CommandScheduler.getInstance().onCommandFinish(
                command -> {
                    Shuffleboard.addEventMarker("Command finished.", command.getName(), EventImportance.kNormal);
                    logger.debug("{} finished.", command.getName());
                }
        );
    }

    public boolean checkRobotIntegrity() {
        logger.info("Starting robot health test...");
        return colorWheelSpinner.checkIntegrity();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutoMode() {
        return autoMode = autoChooser.getSelected();
    }

    public boolean isRobotPaused() {
        if (driverController.getStartButton()) {
            isRobotPaused = !isRobotPaused;
        }

        return isRobotPaused;
    }

}
