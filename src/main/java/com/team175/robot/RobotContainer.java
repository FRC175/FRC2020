package com.team175.robot;

import com.team175.robot.commands.BlinkLED;
import com.team175.robot.commands.LockOntoTarget;
import com.team175.robot.commands.RotateTurretToFieldOrientedCardinal;
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
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
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

    // The robot's subsystems and commands are defined here
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

    private Command autoMode;

    private static RobotContainer instance;

    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int OPERATOR_CONTROLLER_PORT = 1;
    private static final double CONTROLLER_DEADBAND = 0.1;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
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
                        () -> drive.arcadeDrive(
                                driverController.getTriggerAxis(GenericHID.Hand.kRight) - driverController.getTriggerAxis(GenericHID.Hand.kLeft),
                                driverController.getX(GenericHID.Hand.kLeft)
                        ),
                        drive
                ).andThen(() -> drive.arcadeDrive(0, 0), drive)
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by instantiating a {@link
     * GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {
        // Align to target
        new XboxButton(driverController, AdvancedXboxController.Button.X)
                .toggleWhenPressed(new LockOntoTarget(shooter, limelight))
                .whenReleased(() -> shooter.setHoodPosition(0), shooter);

        // Toggle LED
        /*new XboxButton(driverController, AdvancedXboxController.Button.A)
                .toggleWhenPressed(new InstantCommand() {
                    private boolean toggle = true;

                    @Override
                    public void initialize() {
                        toggle = !toggle;
                        limelight.setLED(toggle);
                    }
                });*/

        // Blink LED
        new XboxButton(driverController, AdvancedXboxController.Button.B)
                .whenPressed(new BlinkLED(led, Color.kGreen));

        // Intake control
        new XboxButton(operatorController, AdvancedXboxController.Button.A)
                .whileHeld(() -> intake.setRollerOpenLoop(1), intake)
                .whenPressed(() -> intake.setIndexerOpenLoop(1), intake);

        // Indexer toggle
        new XboxButton(operatorController, AdvancedXboxController.Button.X)
                .toggleWhenPressed(new FunctionalCommand(
                        () -> {},
                        () -> intake.setIndexerOpenLoop(1),
                        (finished) -> intake.setIndexerOpenLoop(0),
                        () -> false,
                        intake
                ));

        new XboxButton(operatorController, AdvancedXboxController.Button.Y)
                .whileHeld(() -> {
                    intake.setIndexerOpenLoop(-1);
                    intake.setRollerOpenLoop(-1);
                }, intake)
                .whenReleased(() -> {
                    intake.setIndexerOpenLoop(0);
                    intake.setRollerOpenLoop(0);
                }, intake);


        // Deploy/retract color wheel device
        new XboxButton(operatorController, AdvancedXboxController.Button.LEFT_BUMPER)
                .whenPressed(() -> colorWheelSpinner.deploy(!colorWheelSpinner.isDeployed()), colorWheelSpinner);

        // Manual color wheel control
        new XboxButton(driverController, AdvancedXboxController.Button.B)
                .whileHeld(() -> colorWheelSpinner.setOpenLoop(1), colorWheelSpinner)
                .whenReleased(() -> colorWheelSpinner.setOpenLoop(0), colorWheelSpinner);

        // Shift gears
        new XboxButton(driverController, AdvancedXboxController.Button.X)
                .whileHeld(() -> drive.setHighGear(true), drive)
                .whenReleased(() -> drive.setHighGear(false), drive);

        // Manual Turret Control
        new XboxButton(operatorController, AdvancedXboxController.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(
                        () -> {
                            shooter.setTurretOpenLoop(operatorController.getX(GenericHID.Hand.kRight));
                            shooter.setShooterOpenLoop(operatorController.getY(GenericHID.Hand.kRight));
                        },
                        shooter
                ))
                .whenReleased(() -> shooter.setTurretOpenLoop(0), shooter);

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

        // Deploy/retract winch
        new XboxButton(driverController, AdvancedXboxController.Button.LEFT_BUMPER)
                .whenPressed(() -> climber.deploy(!climber.isDeployed()), climber);
        // Reset Shooter sensors
        new XboxButton(driverController, AdvancedXboxController.Button.LEFT_BUMPER)
                .whenPressed(shooter::resetSensors, shooter);

        new XboxButton(operatorController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(() ->  shooter.setHoodOpenLoop(1), shooter)
                .whenReleased(() ->  shooter.setHoodOpenLoop(0), shooter);

        // Turret Cardinals
        new XboxButton(driverController, AdvancedXboxController.DPad.UP)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.NORTH));
        new XboxButton(driverController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.EAST));
        new XboxButton(driverController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.SOUTH));
        new XboxButton(driverController, AdvancedXboxController.DPad.LEFT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.WEST));

        new XboxButton(operatorController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(() ->  shooter.setHoodPosition(1), shooter)
                .whenReleased(() ->  shooter.setHoodPosition(0), shooter);

        new XboxButton(operatorController, AdvancedXboxController.Button.Y)
                .whileHeld(() -> shooter.setHoodOpenLoop(1));
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("Do Nothing", null);
        // autoChooser.addOption();
        // Add more auto modes here
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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutoMode() {
        return autoMode = autoChooser.getSelected();
    }

}
