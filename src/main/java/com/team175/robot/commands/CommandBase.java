package com.team175.robot.commands;

import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * CommandBase is an extension to {@link edu.wpi.first.wpilibj2.command.CommandBase} that adds a logger.
 */
public class CommandBase extends edu.wpi.first.wpilibj2.command.CommandBase {

    /**
     * Logger used to document the various actions performed by a subsystem.
     */
    protected final Logger logger = LoggerFactory.getLogger(getClass().getSimpleName());

}
