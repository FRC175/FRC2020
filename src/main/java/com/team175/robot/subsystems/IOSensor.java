package com.team175.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import io.github.oblarg.oblog.annotations.Log;

public class IOSensor extends SubsystemBase {

    private final DigitalInput inSensor;
    private final DigitalInput outSensor;

    @Log
    private int ballCounter = 0;

    private static final int IN_PORT = 4;
    private static final int OUT_PORT = 8;

    private IOSensor() {
        inSensor = new DigitalInput(IN_PORT);
        outSensor = new DigitalInput(OUT_PORT);
    }

    public int getBallCounter() {
        return ballCounter;
    }

    public boolean getInSensor() {
        return !inSensor.get();
    }

    public boolean getOutSensor() {
        return !outSensor.get();
    }

    public void increaseCounter() {
        if (getInSensor()) {
            ballCounter += 1;
        } else if (getOutSensor()) {
            ballCounter -= 1;
        }
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }
}