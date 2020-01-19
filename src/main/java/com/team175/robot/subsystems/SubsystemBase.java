package com.team175.robot.subsystems;

import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class SubsystemBase implements Subsystem, Sendable {

    private final String subsystemName = getClass().getSimpleName();

    /**
     * Constructor
     */
    public SubsystemBase() {
        SendableRegistry.addLW(this, subsystemName, subsystemName);
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Associates a {@link Sendable} with this Subsystem.
     * Also update the child's name.
     *
     * @param name name to give child
     * @param child sendable
     */
    public void addChild(String name, Sendable child) {
        SendableRegistry.addLW(child, subsystemName, name);
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

}