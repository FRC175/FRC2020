package com.team175.robot;

import com.team175.robot.commands.LockOntoTarget;
import com.team175.robot.commands.RotateTurretToFieldOrientedCardinal;
import com.team175.robot.models.AdvancedXboxController;
import com.team175.robot.models.XboxButton;
import com.team175.robot.positions.TurretCardinal;
import com.team175.robot.subsystems.Drive;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
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
public class RobotContainer {

    // The robot's subsystems and commands are defined here
    private final Drive drive;
    private final Limelight limelight;
    private final Shooter shooter;
    private final AdvancedXboxController driverController;
    private final SendableChooser<Command> autoChooser;
    private final Logger logger;

    private Command autoMode;

    private static RobotContainer instance;

    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final double CONTROLLER_DEADBAND = 0.1;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        drive = Drive.getInstance();
        limelight = Limelight.getInstance();
        shooter = Shooter.getInstance();
        driverController = new AdvancedXboxController(DRIVER_CONTROLLER_PORT, CONTROLLER_DEADBAND);
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
        /*new XboxButton(driverController, AdvancedXboxController.Button.X)
                .whileHeld(new RotateTurretToTarget(shooter, limelight));*/
        new XboxButton(driverController, AdvancedXboxController.Button.X)
                .toggleWhenPressed(new LockOntoTarget(shooter, limelight));
        // Blink LED
        new XboxButton(driverController, AdvancedXboxController.Button.Y)
                .whenPressed(new InstantCommand(limelight::blinkLED, limelight)
                        .andThen(new WaitCommand(1))
                        .andThen(limelight::defaultLED, limelight));
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
        // Manual Turret Control
        new XboxButton(driverController, AdvancedXboxController.Button.RIGHT_BUMPER)
                .whileHeld(new RunCommand(
                        () -> shooter.setTurretOpenLoop(driverController.getX(GenericHID.Hand.kRight)),
                        shooter
                ))
                .whenReleased(() -> shooter.setTurretOpenLoop(0), shooter);
        // Reset sensors
        new XboxButton(driverController, AdvancedXboxController.Button.LEFT_BUMPER)
                .whenPressed(
                        () -> {
                            drive.resetSensors();
                            shooter.resetSensors();
                        },
                        drive,
                        shooter
                );
        // Turret Cardinals
        /*new XboxButton(driverController, AdvancedXboxController.DPad.UP)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.NORTH), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.EAST), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.SOUTH), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.LEFT)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.WEST), shooter);*/
        new XboxButton(driverController, AdvancedXboxController.DPad.UP)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.NORTH));
        new XboxButton(driverController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.EAST));
        new XboxButton(driverController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.SOUTH));
        new XboxButton(driverController, AdvancedXboxController.DPad.LEFT)
                .whenPressed(new RotateTurretToFieldOrientedCardinal(drive, shooter, TurretCardinal.WEST));
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
