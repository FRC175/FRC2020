package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PWM;
import io.github.oblarg.oblog.annotations.Log;

public final class Intake extends SubsystemBase {

    private final VictorSPX roller, indexerHorizontal, indexerVertical;
    private final DoubleSolenoid deployer;

    private final DigitalInput intakeInSensor;
    private final DigitalInput intakeOutSensor;
    private final PWM pwm;

    private static final int PCM_PORT = 17;
    private static final int ROLLER_PORT = 6;
    private static final int INDEXER_HORIZONTAL_PORT = 8;
    private static final int INDEXER_VERTICAL_PORT = 9;
    private static final int DEPLOYER_FORWARD_CHANNEL = 4;
    private static final int DEPLOYER_REVERSE_CHANNEL = 5;
    private static final int INTAKE_IN_PORT = 4;
    private static final int INTAKE_OUT_PORT = 8;
    private static final int PWM_PORT = 0;

    private static Intake instance;

    private Intake() {
        roller = new VictorSPX(ROLLER_PORT);
        indexerHorizontal = new VictorSPX(INDEXER_HORIZONTAL_PORT);
        indexerVertical = new VictorSPX(INDEXER_VERTICAL_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        intakeInSensor = new DigitalInput(INTAKE_IN_PORT);
        intakeOutSensor = new DigitalInput(INTAKE_OUT_PORT);
        pwm = new PWM(PWM_PORT);
        configureVictors();
    }

    /*public boolean getIO() {
        return !IOSensor.get();
    }

    public void senseObject() {
        pwm.setRaw(getIO() ? 255 : 0);
    }*/

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }

        return instance;
    }

    private void configureVictors() {
        roller.configFactoryDefault();
        indexerHorizontal.configFactoryDefault();
        indexerVertical.configFactoryDefault();
    }

    public void setRollerOpenLoop(double demand) {
        roller.set(ControlMode.PercentOutput, demand);
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
