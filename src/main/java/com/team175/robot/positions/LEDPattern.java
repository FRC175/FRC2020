package com.team175.robot.positions;

public enum LEDPattern {

    RAINBOW(-0.09),
    CONFETTI(-0.87),
    TWINKLES(-0.53),
    COLOR_WAVE(-0.45),
    BLINK_RED(-0.11),
    BLINK_BLUE(-0.09),
    BLINK_YELLOW(-0.07),
    BLINK_WHITE(-0.05);

    private final double value;

    LEDPattern(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

}
