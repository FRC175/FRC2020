package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public final class Shooter extends SubsystemBase {

    private final TalonSRX turret;

    private static final int TURRET_PORT = 5;

    private static Shooter instance;

    private Shooter() {
        turret = new TalonSRX(TURRET_PORT);

        configureTalons();
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

    private void configureTalons() {
        turret.configFactoryDefault();
        turret.setInverted(true);
    }

    public void setTurretOpenLoop(double demand) {
        turret.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
