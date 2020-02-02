package com.team175.robot.models;

/**
 * Gains holds PID gains (Kp, Ki, Kd, Kf) and acceleration and cruise velocity data for closed loop control.
 */
public class Gains {

    /**
     * PIDF gains
     */
    private final double kP, kI, kD, kF;
    /**
     * Motion magic parameters
     */
    private final int acceleration, cruiseVelocity;

    public Gains(double kP, double kI, double kD, double kF, int acceleration, int cruiseVelocity) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        this.acceleration = acceleration;
        this.cruiseVelocity = cruiseVelocity;
    }

    public Gains(double kP, double kI, double kD) {
        this(kP, kI, kD, 0, 0, 0);
    }

    public double getKp() {
        return kP;
    }

    public double getKi() {
        return kI;
    }

    public double getKd() {
        return kD;
    }

    public double getKf() {
        return kF;
    }

    public int getAcceleration() {
        return acceleration;
    }

    public int getCruiseVelocity() {
        return cruiseVelocity;
    }

    public double[] toArray() {
        return new double[]{kP, kI, kD, kF, acceleration, cruiseVelocity};
    }

}