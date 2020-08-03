package com.team175.robot.positions;

public enum LEDColor {

    RED(0.61),
    YELLOW(0.67),
    GREEN(0.77),
    BLUE(0.81),
    OFF(0.99),
    SUCCESS(GREEN.value),
    ERROR(RED.value),
    DEFAULT(YELLOW.value),
    BALL_CAPACITY_REACHED(YELLOW.value);

    private final double value;

    LEDColor(double value) {
        this.value = value;
    }

    public double getValue() {
        return value;
    }

}
