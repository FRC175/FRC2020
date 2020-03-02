package com.team175.robot.utils;

import edu.wpi.first.wpilibj.geometry.Rotation2d;

public final class SensorUnits {

    // Helper class
    private SensorUnits() {
    }

    public static int degreesToCounts(Rotation2d heading, final int countsPerRevolution) {
        return (int) (heading.getDegrees() * (countsPerRevolution / 360.0));
    }

    public static Rotation2d countsToDegrees(double position, final int countsPerRevolution) {
        return Rotation2d.fromDegrees(position * (360.0 / countsPerRevolution));
    }

    public static double countsToRotations(double position, final int countsPerRevolution) {
        return position / countsPerRevolution;
    }

    public static int rpmToTalonVelocity(double rpm, final int countsPerRevolution) {
        return (int) (rpm * (double) countsPerRevolution * (1.0 / 600.0));
    }

    public static double talonVelocityToRPM(double velocity, final int countsPerRevolution) {
        return velocity * (1.0 / countsPerRevolution) * 600.0;
    }

}
