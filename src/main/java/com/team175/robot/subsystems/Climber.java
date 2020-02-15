package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.Solenoid;
import io.github.oblarg.oblog.annotations.Log;

public final class Climber extends SubsystemBase {

    private final VictorSPX winchMaster, winchSlave;
    private final Solenoid deployer;

    private static final int PCM_PORT = 17;
    private static final int WINCH_MASTER_PORT = 14;
    private static final int WINCH_SLAVE_PORT = 15;
    private static final int DEPLOYER_CHANNEL = 7;

    private static Climber instance;

    private Climber() {
        winchMaster = new VictorSPX(WINCH_MASTER_PORT);
        winchSlave = new VictorSPX(WINCH_SLAVE_PORT);
        deployer = new Solenoid(PCM_PORT, DEPLOYER_CHANNEL);
        configureVictors();
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private void configureVictors() {
        winchMaster.configFactoryDefault();
        winchSlave.configFactoryDefault();
        winchSlave.follow(winchMaster);
    }

    public void setWinchOpenLoop(double demand) {
        winchMaster.set(ControlMode.PercentOutput, demand);
    }

    public void deploy(boolean deploy) {
        deployer.set(deploy);
    }

    public void deploy() {
        deploy(true);
    }

    public void retract() {
        deploy(false);
    }

    @Log
    public boolean isDeployed() {
        return deployer.get();
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
