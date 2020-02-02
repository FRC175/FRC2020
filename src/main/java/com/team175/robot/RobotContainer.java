package com.team175.robot;

import com.team175.robot.models.AldrinXboxController;
import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import io.github.oblarg.oblog.Logger;

/**
 * RobotContainer is where the bulk of the robot should be declared.  Since Command-based is a "declarative" paradigm,
 * very little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler
 * calls). Instead, the structure of the robot (including subsystems, commands, and button mappings) should be declared
 * here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here
    private final Drive drive;
    private final AldrinXboxController controller;
    private final Logger oblogLogger;

    private Command autoCommand;

    private static RobotContainer instance;

    private static final int CONTROLLER_PORT = 0;

    /**
     * The container for the robot.  Contains subsystems, OI devices, and commands.
     */
    private RobotContainer() {
        drive = Drive.getInstance();
        controller = new AldrinXboxController(CONTROLLER_PORT);
        oblogLogger = new Logger();

        configureDefaultCommands();
        configureButtonBindings();
        configureOblogLogger();
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
                        () -> {},
                        () -> drive.arcadeDrive(
                                controller.getTriggerAxis(GenericHID.Hand.kRight)
                                        - controller.getTriggerAxis(GenericHID.Hand.kLeft),
                                controller.getX(GenericHID.Hand.kLeft)
                        ),
                        (interrupted) -> drive.setOpenLoop(0, 0),
                        () -> false,
                        drive
                )
        );
    }

    /**
     * Use this method to define your button->command mappings.  Buttons can be created by instantiating a {@link
     * GenericHID} or one of its subclasses ({@link edu.wpi.first.wpilibj.Joystick Joystick} or {@link XboxController}),
     * and then passing it to a {@link edu.wpi.first.wpilibj2.command.button.JoystickButton JoystickButton}.
     */
    private void configureButtonBindings() {

    }

    private void configureOblogLogger() {
        Logger.configureLoggingAndConfig(this, false);
    }

    public void updateOblogLogger() {
        oblogLogger.updateEntries();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }

}
