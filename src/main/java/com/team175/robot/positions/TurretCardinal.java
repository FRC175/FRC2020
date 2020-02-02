package com.team175.robot.positions;

public enum TurretCardinal {

    NORTH(0),
    EAST(900),
    SOUTH(1800),
    WEST(2750);

    private final int angle;

    TurretCardinal(int angle) {
        this.angle = angle;
    }

    public int toDegrees() {
        return angle;
    }

}
