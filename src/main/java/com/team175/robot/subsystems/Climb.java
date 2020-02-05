package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public final class Climb extends SubsystemBase {

    private final TalonSRX hook;
    private final VictorSPX lift;
    private final VictorSPX climb;

    private static final int HOOK_PORT = 10;
    private static final int LIFT_PORT = 14;
    private static final int CLIMB_PORT = 15;

    private static Climb instance;

    private Climb() {
        hook = new TalonSRX(HOOK_PORT);
        lift = new VictorSPX(LIFT_PORT);
        climb = new VictorSPX(CLIMB_PORT);
        configureTalons();
    }

    public static Climb getInstance() {
        if (instance == null) {
            instance = new Climb();
        }

        return instance;
    }

    private void configureTalons() {
        // Configuration
        hook.configFactoryDefault();
        lift.configFactoryDefault();
        climb.configFactoryDefault();
    }

    public void setOpenLoop(double demand) {
        hook.set(ControlMode.PercentOutput, demand);
        lift.set(ControlMode.PercentOutput, demand);
        climb.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
