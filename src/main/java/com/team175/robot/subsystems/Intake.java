package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Intake represents the intake and ball indexing mechanism. It is composed of 1 NEO 550 brushless motor, 2 775-Pro
 * motors (controlled by Victor SPX motor controllers), 2 pneumatic pistons, and 2 IR sensors.
 */
public final class Intake extends SubsystemBase {

    private final CANSparkMax roller;
    private final VictorSPX indexerHorizontal, indexerVertical;
    private final DoubleSolenoid deployer;
    private final DigitalInput intakeInSensor, intakeOutSensor;

    private boolean isBallPickedUp;

    private static final int PCM_PORT = 17;
    private static final int ROLLER_PORT = 6;
    private static final int INDEXER_HORIZONTAL_PORT = 8;
    private static final int INDEXER_VERTICAL_PORT = 9;
    private static final int DEPLOYER_FORWARD_CHANNEL = 4;
    private static final int DEPLOYER_REVERSE_CHANNEL = 5;
    private static final int INTAKE_IN_PORT = 4;
    private static final int INTAKE_OUT_PORT = 8;
    private static final double BALL_PICKED_UP_CURRENT = 10;

    private static Intake instance;

    private Intake() {
        roller = new CANSparkMax(ROLLER_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        indexerHorizontal = new VictorSPX(INDEXER_HORIZONTAL_PORT);
        indexerVertical = new VictorSPX(INDEXER_VERTICAL_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        intakeInSensor = new DigitalInput(INTAKE_IN_PORT);
        intakeOutSensor = new DigitalInput(INTAKE_OUT_PORT);
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

    private void configureSparkMax() {
        roller.restoreFactoryDefaults();
    }

    private void configureVictors() {
        indexerHorizontal.configFactoryDefault();
        indexerVertical.configFactoryDefault();
    }

    public void setRollerOpenLoop(double demand) {
        roller.set(demand);
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
        return isBallPickedUp;
    }

    @Override
    public void periodic() {
        /*if (roller.getOutputCurrent() >= BALL_PICKED_UP_CURRENT) {
            // Blink LED
            isBallPickedUp = true;
        } else {
            isBallPickedUp = false;
        }*/
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
