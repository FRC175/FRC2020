package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public final class ControlPanel extends SubsystemBase {

    private final TalonSRX controlPanelWheel;

    private static final int WHEEL_PORT = 5;

    private static ControlPanel instance;

    private ControlPanel() {
        controlPanelWheel = new TalonSRX(WHEEL_PORT);
        configureTalons();
    }

    public static ControlPanel getInstance() {
        if (instance == null) {
            instance = new ControlPanel();
        }

        return instance;
    }

    private void configureTalons() {
        // Configuration
        controlPanelWheel.configFactoryDefault();
    }

    public void setOpenLoop(double demand) {
        controlPanelWheel.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
