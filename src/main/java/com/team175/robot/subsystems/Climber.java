package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public final class Climber extends SubsystemBase {

    private final TalonSRX winch;
    private final DoubleSolenoid deployer;

    private static final int PCM_PORT = 17;
    private static final int WINCH_PORT = 10;
    private static final int DEPLOYER_FORWARD_CHANNEL = 4;
    private static final int DEPLOYER_REVERSE_CHANNEL = 5;

    private static Climber instance;

    private Climber() {
        winch = new TalonSRX(WINCH_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        configureTalons();
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private void configureTalons() {
        winch.configFactoryDefault();
    }

    public void setWinchOpenLoop(double demand) {
        winch.set(ControlMode.PercentOutput, demand);
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
