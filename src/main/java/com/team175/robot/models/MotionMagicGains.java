package com.team175.robot.models;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilder;

public class MotionMagicGains extends Gains implements Sendable {

    private final TalonSRX talon;

    private double kF;
    private int acceleration, cruiseVelocity;

    public MotionMagicGains(double kP, double kI, double kD, double kF, int acceleration, int cruiseVelocity, TalonSRX talon) {
        super(kP, kI, kD);
        this.talon = talon;
        setKp(super.kP);
        setKi(super.kI);
        setKd(super.kD);
        setKf(kF);
        setAcceleration(acceleration);
        setCruiseVelocity(cruiseVelocity);
    }

    private void setKp(double kP) {
        super.kP = kP;
        talon.config_kP(0, super.kP);
    }

    private void setKi(double kI) {
        super.kI = kI;
        talon.config_kI(0, super.kI);
    }

    private void setKd(double kD) {
        super.kD = kD;
        talon.config_kD(0, super.kD);
    }

    private void setKf(double kF) {
        this.kF = kF;
        talon.config_kF(0, this.kF);
    }

    private void setAcceleration(int acceleration) {
        this.acceleration = acceleration;
        talon.configMotionAcceleration(this.acceleration);
    }

    private void setCruiseVelocity(int cruiseVelocity) {
        this.cruiseVelocity = cruiseVelocity;
        talon.configMotionCruiseVelocity(this.cruiseVelocity);
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

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Kp", super::getKp, this::setKp);
        builder.addDoubleProperty("Ki", super::getKi, this::setKi);
        builder.addDoubleProperty("Kd", super::getKd, this::setKd);
        builder.addDoubleProperty("Kf", this::getKf, this::setKf);
        builder.addDoubleProperty("Acceleration", this::getAcceleration, (value) -> setAcceleration((int) value));
        builder.addDoubleProperty("Cruise Velocity", this::getCruiseVelocity, (value) -> setCruiseVelocity((int) value));
    }

}
