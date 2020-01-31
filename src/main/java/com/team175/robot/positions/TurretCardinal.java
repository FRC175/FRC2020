package com.team175.robot.positions;

public enum TurretCardinal {

    NORTH(0),
    WEST(90),
    SOUTH(180),
    EAST(270);

    private final int angle;

    TurretCardinal(int angle) {
        this.angle = angle;
    }

    public int toDegrees() {
        return angle;
    }

}
