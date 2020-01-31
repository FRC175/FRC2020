package com.team175.robot;

import com.team175.robot.commands.ManualTurretControl;
import com.team175.robot.commands.RotateTurretToTarget;
import com.team175.robot.models.AdvancedXboxController;
import com.team175.robot.models.XboxButton;
import com.team175.robot.positions.TurretCardinal;
import com.team175.robot.subsystems.Drive;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;

/**
 * RobotContainer is where the bulk of the robot should be declared.  Since Command-based is a "declarative" paradigm,
 * very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public final class RobotContainer {

    // The robot's subsystems and commands are defined here
    private final Drive drive;
    private final Shooter shooter;
    private final Limelight limelight;
    private final AdvancedXboxController driverController;
    // , operatorController;
    private final SendableChooser<Command> autoChooser;

    private Command autoMode;

    private static RobotContainer instance;

    private static final int DRIVER_CONTROLLER_PORT = 0;
    private static final int OPERATOR_CONTROLLER_PORT = 1;
    private static final double CONTROLLER_DEADBAND = 0.10;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        drive = Drive.getInstance();
        shooter = Shooter.getInstance();
        limelight = Limelight.getInstance();

        driverController = new AdvancedXboxController(DRIVER_CONTROLLER_PORT, CONTROLLER_DEADBAND);
        // operatorController = new AdvancedXboxController(OPERATOR_CONTROLLER_PORT, CONTROLLER_DEADBAND);

        autoChooser = new SendableChooser<>();

        configureDefaultCommands();
        configureButtonBindings();
        configureAutoChooser();
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
                new FunctionalCommand(
                        () -> {
                        },
                        () -> drive.arcadeDrive(
                                driverController.getTriggerAxis(GenericHID.Hand.kRight)
                                        - driverController.getTriggerAxis(GenericHID.Hand.kLeft),
                                driverController.getX(GenericHID.Hand.kLeft)
                        ),
                        (interrupted) -> drive.setOpenLoop(0, 0),
                        () -> false,
                        drive
                )
                /*new RunCommand(
                        () -> drive.arcadeDrive(
                                driverController.getTriggerAxis(GenericHID.Hand.kRight) - driverController.getTriggerAxis(GenericHID.Hand.kLeft),
                                driverController.getX(GenericHID.Hand.kLeft)
                        ),
                        drive
                ).whenFinished(() -> drive.setOpenLoop(0, 0));*/
        );

        // Manual turret control
        shooter.setDefaultCommand(new ManualTurretControl(shooter, () -> driverController.getX(GenericHID.Hand.kRight)));
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by instantiating a {@link
     * GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {
        new XboxButton(driverController, AdvancedXboxController.Button.X)
                .whileHeld(new RotateTurretToTarget(shooter, limelight));
        new XboxButton(driverController, AdvancedXboxController.Button.Y)
                .whenPressed(limelight::blinkLED, limelight);
        new XboxButton(driverController, AdvancedXboxController.Button.Y)
                .whenPressed(() -> limelight.setLED(true), limelight)
                .whenReleased(() -> limelight.setLED(false), limelight);
        new XboxButton(driverController, AdvancedXboxController.DPad.UP)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.NORTH), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.RIGHT)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.EAST), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.DOWN)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.SOUTH), shooter);
        new XboxButton(driverController, AdvancedXboxController.DPad.LEFT)
                .whenPressed(() -> shooter.setTurretCardinal(TurretCardinal.WEST), shooter);
    }

    private void configureAutoChooser() {
        autoChooser.setDefaultOption("No Auto", null);
        // Add more auto modes here
        SmartDashboard.putData("Auto Mode Chooser", autoChooser);
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
