package com.team175.robot.models;

/**
 * Gains holds PID gains (Kp, Ki, Kd) for closed loop control.
 */
public class Gains {

    protected double kP, kI, kD;

    public Gains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
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

}