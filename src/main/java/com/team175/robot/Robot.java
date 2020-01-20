/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.team175.robot;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to each mode, as
 * described in the TimedRobot documentation. If you change the name of this class or the package after creating this
 * project, you must also update the build.gradle file in the project.
 */
public class Robot extends TimedRobot {

    private RobotContainer robotContainer;
    private Command autoCommand;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private final ColorMatch colorMatcher = new ColorMatch();

    private static final Color BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color RED_TARGET = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.524, 0.113);

    /**
     * This method is run when the robot is first started up and should be used for any initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate the RobotContainer. This will perform all our button bindings, and put our autonomous chooser on
        // the dashboard.
        robotContainer = new RobotContainer();

        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(GREEN_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        colorMatcher.addColorMatch(YELLOW_TARGET);
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

        /**
         * The method GetColor() returns a normalized color value from the sensor and can be useful if outputting the
         * color to an RGB LED or similar. To read the raw color, use GetRawColor().
         *
         * The color sensor works best when within a few inches from an object in well lit conditions (the built in
         * LED is a big help here!). The farther an object is the more light from the surroundings will bleed into the
         * measurements and make it difficult to accurately determine its color.
         */
        Color detectedColor = colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = colorSensor.getIR();

        String color;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == BLUE_TARGET) {
            color = "Blue";
        } else if (match.color == RED_TARGET) {
            color = "Red";
        } else if (match.color == GREEN_TARGET) {
            color = "Green";
        } else if (match.color == YELLOW_TARGET) {
            color = "Yellow";
        } else {
            color = "Unknown";
        }


        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", color);

        /**
         * In addition to RGB IR values, the color sensor can also return an infrared proximity value. The chip
         * contains an IR led which will emit IR pulses and measure the intensity of the return. When an object is
         * close the value of the proximity will be large (max 2047 with default settings) and will approach zero when
         * the object is far away.
         *
         * Proximity can be used to roughly approximate the distance of an object or provide a threshold for when an
         * object is close enough to provide accurate color values.
         */
        int proximity = colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);
    }

    /**
     * This method is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        autoCommand = robotContainer.getAutonomousCommand();

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
    }

    /**
     * This method is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }

}
