package com.team175.robot.appenders;

import ch.qos.logback.classic.Level;
import ch.qos.logback.classic.spi.ILoggingEvent;
import ch.qos.logback.core.AppenderBase;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * DriverStationAppender is a logback appender that translates SLF4J log errors or warnings to their respective driver
 * station calls.
 */
public class DriverStationAppender extends AppenderBase<ILoggingEvent> {

    @Override
    protected void append(ILoggingEvent event) {
        if (event.getLevel().isGreaterOrEqual(Level.WARN)) {
            if (event.getLevel().isGreaterOrEqual(Level.ERROR)) {
                DriverStation.reportError(event.getFormattedMessage(), false);
            } else {
                DriverStation.reportWarning(event.getFormattedMessage(), false);
            }
        }
    }

}
