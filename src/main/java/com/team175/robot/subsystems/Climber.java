package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Solenoid;
import io.github.oblarg.oblog.annotations.Log;

public final class Climber extends SubsystemBase {

    private final CANSparkMax winchMaster, winchSlave;
    private final CANPIDController winchPIDController;
    private final Solenoid deployer;
    private final DigitalInput digital;
    private final AnalogInput analog;

    private static final int PCM_PORT = 18;
    private static final int WINCH_MASTER_PORT = 10;
    private static final int WINCH_SLAVE_PORT = 3;
    private static final int DEPLOYER_CHANNEL = 7;

    private static Climber instance;

    private Climber() {
        winchMaster = new CANSparkMax(WINCH_MASTER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        winchSlave = new CANSparkMax(WINCH_SLAVE_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        winchPIDController = winchMaster.getPIDController();
        configureSparkMaxes();
        deployer = new Solenoid(PCM_PORT, DEPLOYER_CHANNEL);
        digital = new DigitalInput(1);
        analog = new AnalogInput(1);
    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }

        return instance;
    }

    private void configureSparkMaxes() {
        winchMaster.restoreFactoryDefaults();
        winchSlave.restoreFactoryDefaults();
        winchSlave.follow(winchMaster);
    }

    public void setWinchOpenLoop(double demand) {
        winchMaster.set(demand);
    }

    public void deploy(boolean deploy) {
        logger.info("{} climber", deploy ? "Deploying" : "Retracting");
        deployer.set(deploy);
    }

    public void deploy() {
        deploy(true);
    }

    public void retract() {
        deploy(false);
    }

    // @Log
    public boolean isDeployed() {
        return deployer.get();
    }

    @Log
    public boolean digitalInputValue() {
        return digital.get();
    }

    @Log
    public double analogInputValue() {
        return analog.getVoltage();
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
