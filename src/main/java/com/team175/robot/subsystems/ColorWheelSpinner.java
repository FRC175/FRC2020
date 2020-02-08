package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import io.github.oblarg.oblog.annotations.Log;

public final class ColorWheelSpinner extends SubsystemBase {

    private final TalonSRX spinner;
    private final DoubleSolenoid deployer;

    private static final int PCM_PORT = 17;
    private static final int SPINNER_PORT = 5;
    private static final int DEPLOYER_FORWARD_CHANNEL = 0;
    private static final int DEPLOYER_REVERSE_CHANNEL = 1;

    private static ColorWheelSpinner instance;

    private ColorWheelSpinner() {
        spinner = new TalonSRX(SPINNER_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        configureTalons();
    }

    public static ColorWheelSpinner getInstance() {
        if (instance == null) {
            instance = new ColorWheelSpinner();
        }

        return instance;
    }

    private void configureTalons() {
        spinner.configFactoryDefault();
    }

    public void setOpenLoop(double demand) {
        spinner.set(ControlMode.PercentOutput, demand);
    }

    public void deploy(boolean deploy) {
        deployer.set(deploy ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    public void deploy() {
        deploy(true);
    }

    public void retract() {
        deploy(false);
    }

    @Log
    public boolean isDeployed() {
        return deployer.get() == DoubleSolenoid.Value.kForward;
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
