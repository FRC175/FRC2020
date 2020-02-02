package com.team175.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * LogCommand uses {@link org.slf4j.Logger} to log a debug message.
 */
public final class LogCommand extends CommandBase {

    private final Logger logger = LoggerFactory.getLogger(getClass().getSimpleName());
    private final String message;

    public LogCommand(String message) {
        this.message = message;
    }

    @Override
    public void initialize() {
        logger.debug(message);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}