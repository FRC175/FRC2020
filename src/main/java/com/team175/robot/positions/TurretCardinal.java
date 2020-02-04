package com.team175.robot.positions;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public enum TurretCardinal {

    NORTH(Rotation2d.fromDegrees(0)),
    EAST(Rotation2d.fromDegrees(90)),
    SOUTH(Rotation2d.fromDegrees(180)),
    WEST(Rotation2d.fromDegrees(270));

    private final Rotation2d heading;

    TurretCardinal(Rotation2d heading) {
        this.heading = heading;
    }

    public Rotation2d toRotation2d() {
        return heading;
    }

}
