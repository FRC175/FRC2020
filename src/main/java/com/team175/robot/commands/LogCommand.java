package com.team175.robot.commands;

/**
 * LogCommand uses {@link org.slf4j.Logger} to log a debug message.
 */
public final class LogCommand extends CommandBase {

    private final String message;

    public LogCommand(String message) {
        this.message = message;
    }

    @Override
    public void initialize() {
        logger.debug(message);
        super.initialize();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}