package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import io.github.oblarg.oblog.annotations.Log;

public final class Intake extends SubsystemBase {

    private final VictorSPX indexerHorizontal, indexerVertical;
    private final DoubleSolenoid deployer;
    private final CANSparkMax roller;

    private static final int PCM_PORT = 17;
    private static final int ROLLER_PORT = 6;
    private static final int INDEXER_HORIZONTAL_PORT = 8;
    private static final int INDEXER_VERTICAL_PORT = 9;
    private static final int DEPLOYER_FORWARD_CHANNEL = 4;
    private static final int DEPLOYER_REVERSE_CHANNEL = 5;

    private static Intake instance;

    private Intake() {
        roller = new CANSparkMax(ROLLER_PORT, MotorType.kBrushless);
        indexerHorizontal = new VictorSPX(INDEXER_HORIZONTAL_PORT);
        indexerVertical = new VictorSPX(INDEXER_VERTICAL_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        configureVictors();
        configureSparkMax();
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private void configureVictors() {
        indexerHorizontal.configFactoryDefault();
        indexerVertical.configFactoryDefault();
    }

    private void configureSparkMax() {
        roller.restoreFactoryDefaults();
    }

    public void setRollerOpenLoop(double demand) {
        roller.set(demand);
        indexerHorizontal.set(ControlMode.PercentOutput, demand);
        indexerVertical.set(ControlMode.PercentOutput, demand);
    }

    public void setIndexerOpenLoop(double demand) {
        indexerHorizontal.set(ControlMode.PercentOutput, demand);
        indexerVertical.set(ControlMode.PercentOutput, demand);
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

    @Log
    public boolean isBallPickedUp() {
        return true;
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
