package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

public final class Ball extends SubsystemBase {

    private final VictorSPX ballIntake;
    private final VictorSPX ballStage2;
    private final VictorSPX ballStage3;

    private static final int INTAKE_PORT = 6;
    private static final int STAGE2_PORT = 8;
    private static final int STAGE3_PORT = 9;

    private static Ball instance;

    private Ball() {
        ballIntake = new VictorSPX(INTAKE_PORT);
        ballStage2 = new VictorSPX(STAGE2_PORT);
        ballStage3 = new VictorSPX(STAGE3_PORT);
        configureVictors();
    }

    public static Ball getInstance() {
        if (instance == null) {
            instance = new Ball();
        }

        return instance;
    }

    private void configureVictors() {
        // Configuration
        ballIntake.configFactoryDefault();
        ballStage2.configFactoryDefault();
        ballStage3.configFactoryDefault();
    }

    public void setOpenLoop(double demand) {
        ballIntake.set(ControlMode.PercentOutput, demand);
        ballStage2.set(ControlMode.PercentOutput, demand);
        ballStage3.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
