/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team175.robot;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import io.github.oblarg.oblog.Logger;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public final class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command autoCommand;

    // FIXME: Logging can create latency between robot and driver station, so enable once only important variables are logged.
    // NOTE: This is for logging to the shuffleboard, not the console or log file.
    private static final boolean IS_LOGGING_ENABLED = false;

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        robotContainer = RobotContainer.getInstance();
        if (IS_LOGGING_ENABLED) {
            Logger.configureLoggingAndConfig(robotContainer, false);
        }
    }

    /**
     * This method is called every robot packet, no matter the mode. Use this for items like diagnostics that you want
     * ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and SmartDashboard integrated
     * updating.
     */
    @Override
    public void robotPeriodic() {
        // Run the Scheduler. This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        if (IS_LOGGING_ENABLED) {
            Logger.updateEntries();
        }
    }

    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        Drive.getInstance().resetSensors();
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // Configure subsystems for autonomous
        robotContainer.configureAutoInit();

        // Initialize autonomous mode from shuffleboard input
        autoCommand = robotContainer.getAutoMode();
        if (autoCommand != null) {
            autoCommand.schedule();
        }
    }

    /**
     * This method is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        // Configure subsystems for teleop
        robotContainer.configureTeleopInit();

        // Make sure that the autonomous stops running when teleop starts running
        if (autoCommand != null) {
            autoCommand.cancel();
        }
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        // Cancel all running commands at the start of test mode
        CommandScheduler.getInstance().cancelAll();
        robotContainer.checkRobotIntegrity();
    }

    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
