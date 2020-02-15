package com.team175.robot.subsystems;

import com.ctre.phoenix.CANifier;
import edu.wpi.first.wpilibj.util.Color;

public final class LED extends SubsystemBase {

    private final CANifier controller;

    private Color colorSetpoint;

    private static final int CANIFIER_PORT = 0;

    private static LED instance;

    private LED() {
        controller = new CANifier(CANIFIER_PORT);
        configCANifier();
    }

    public static LED getInstance() {
        if (instance == null) {
            instance = new LED();
        }

        return instance;
    }

    private void configCANifier() {
        controller.configFactoryDefault();
        setColor(Color.kBlue);
    }

    public void setColor(Color color) {
        colorSetpoint = color;
        controller.setLEDOutput(color.red, CANifier.LEDChannel.LEDChannelA);
        controller.setLEDOutput(color.green, CANifier.LEDChannel.LEDChannelB);
        controller.setLEDOutput(color.blue, CANifier.LEDChannel.LEDChannelC);
    }

    public void blinkColor(Color color) {
    }

    public Color getColor() {
        return colorSetpoint;
    }

    @Override
    public void resetSensors() {
        setColor(Color.kBlack);
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
