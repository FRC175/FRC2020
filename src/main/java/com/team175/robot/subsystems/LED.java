package com.team175.robot.subsystems;

import com.team175.robot.positions.LEDColor;
import com.team175.robot.positions.LEDPattern;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.Timer;
import io.github.oblarg.oblog.annotations.Config;

/**
 * FIXME: Not sure if this works...
 */
public final class LED extends SubsystemBase {

    private final Spark blinkin;

    private double blinkinValue, startTime, time, timeInPattern;
    private LEDState state;

    private static final int BLINKIN_PORT = 1;
    private static final double DEFAULT_TIME_IN_PATTERN = 1;
    private static final double BLINK_CYCLE_DURATION = 0.2; // seconds for each blink cycle

    private static LED instance;

    private enum LEDState {
        TIMED,
        BLINK,
        OTHER;
    }

    private LED() {
        blinkin = new Spark(BLINKIN_PORT);
        state = LEDState.OTHER;
        setColor(LEDColor.DEFAULT);
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    @Config
    private void setValue(double value) {
        blinkinValue = value;
        blinkin.set(blinkinValue);
    }

    public void setColor(LEDColor color) {
        setValue(color.getValue());
    }

    public void blinkColor(LEDColor color, double timeInPattern) {
        setColor(color);
        this.timeInPattern = timeInPattern;
        startTime = Timer.getFPGATimestamp();
        state = LEDState.TIMED;
    }

    public void blinkColor(LEDColor color) {
        blinkColor(color, DEFAULT_TIME_IN_PATTERN);
    }

    public void setPattern(LEDPattern pattern) {
        setValue(pattern.getValue());
    }

    public void timePattern(LEDPattern pattern, double timeInPattern) {
        setPattern(pattern);
        this.timeInPattern = timeInPattern;
        startTime = Timer.getFPGATimestamp();
        state = LEDState.TIMED;
    }

    public void timePattern(LEDPattern pattern) {
        timePattern(pattern, DEFAULT_TIME_IN_PATTERN);
    }

    @Override
    public void periodic() {
        switch (state) {
            case BLINK:
                time = Timer.getFPGATimestamp() - startTime;

                if (time > timeInPattern) {
                    setColor(LEDColor.DEFAULT);
                    state = LEDState.OTHER;
                } else {
                    // Every 0.1 s, either turn on or off the LED
                    // 0 s -> on, 0.1 s -> off, 0.2 s -> on...
                    // A blink cycle is 0.2 s
                    int cycleNum = (int) (time / (BLINK_CYCLE_DURATION / 2.0));
                    if ((cycleNum % 2) == 0) {
                        setValue(blinkinValue);
                    } else {
                        setColor(LEDColor.OFF);
                    }
                }

                break;
            case TIMED:
                time = Timer.getFPGATimestamp() - startTime;

                if (time > timeInPattern) {
                    setColor(LEDColor.DEFAULT); // default
                    state = LEDState.OTHER;
                }

                break;
            case OTHER:
                break;
        }
    }

    @Override
    public void resetSensors() {
        setColor(LEDColor.DEFAULT); // default
    }

    @Override
    public boolean checkIntegrity() {
        return true;
    }

}
