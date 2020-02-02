package com.team175.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import io.github.oblarg.oblog.Loggable;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.Supplier;

/**
 * SubsystemBase is the base for all the different subsystems that make up the robot. Each subsystem must implement
 * automated integrity checking of necessary components and reset of sensor(s). A subsystem must only have one
 * instance, which is implemented by the singleton design pattern (after all, a robot can only have one drivetrain).
 */
public abstract class SubsystemBase implements Subsystem, Sendable, Loggable {

    /**
     * Logger used to document the various actions performed by a subsystem.
     */
    protected final Logger logger = LoggerFactory.getLogger(getClass().getSimpleName());

    /**
     * Map for holding telemetry data of a subsystem. Meant to be defined in individual subsystem.
     */
    protected final Map<String, Supplier> telemetry = new LinkedHashMap<>();

    /**
     * Used to make interaction with {@link SendableRegistry} easier.
     */
    private final String subsystemName = getClass().getSimpleName();

    /**
     * Constructor
     */
    public SubsystemBase() {
        SendableRegistry.addLW(this, subsystemName, subsystemName);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none", null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none", null);
    }

    /**
     * Resets the sensors of a subsystem to their initial values (e.g., set encoders to zero units).
     */
    public abstract void resetSensors();

    /**
     * Performs a set of tests to verify a subsystem is behaving properly.
     *
     * @return Whether the subsystem passed all the tests
     */
    public abstract boolean checkIntegrity();

}