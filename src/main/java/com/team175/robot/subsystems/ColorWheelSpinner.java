package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import com.team175.robot.models.MotionMagicGains;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import io.github.oblarg.oblog.annotations.Log;

public final class ColorWheelSpinner extends SubsystemBase {

    private final TalonSRX spinner;
    private final DoubleSolenoid deployer;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;
    private final MotionMagicGains gains;

    private static final int PCM_PORT = 17;
    private static final int SPINNER_PORT = 5;
    private static final int DEPLOYER_FORWARD_CHANNEL = 2;
    private static final int DEPLOYER_REVERSE_CHANNEL = 3;
    private static final int COUNTS_TO_SPIN_WHEEL = 10000; // TODO: Fix
    private static final int ALLOWED_POSITION_DEVIATION = 50;
    private static final Color BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color RED_TARGET = ColorMatch.makeColor(0.415, 0.412, 0.185);
    private static final Color YELLOW_TARGET = ColorMatch.makeColor(0.300, 0.529, 0.170);

    private static ColorWheelSpinner instance;

    private ColorWheelSpinner() {
        spinner = new TalonSRX(SPINNER_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        colorMatcher = new ColorMatch();
        configureTalons();
        gains = new MotionMagicGains(1.5, 0, 0, 0, 0, 0, spinner);
        configureColorSensor();
    }

    public static ColorWheelSpinner getInstance() {
        if (instance == null) {
            instance = new ColorWheelSpinner();
        }

        return instance;
    }

    private void configureTalons() {
        spinner.configFactoryDefault();
        spinner.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        spinner.setSelectedSensorPosition(0);
    }

    private void configureColorSensor() {
        colorMatcher.addColorMatch(BLUE_TARGET);
        colorMatcher.addColorMatch(GREEN_TARGET);
        colorMatcher.addColorMatch(RED_TARGET);
        colorMatcher.addColorMatch(YELLOW_TARGET);
    }

    public void setOpenLoop(double demand) {
        spinner.set(ControlMode.PercentOutput, demand);
    }

    private void setPosition(int position) {
        spinner.set(ControlMode.Position, position);
    }

    public void spinWheel() {
        setPosition(COUNTS_TO_SPIN_WHEEL);
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
    private double getRed() {
        return colorSensor.getColor().red;
    }

    @Log
    private double getGreen() {
        return colorSensor.getColor().green;
    }

    @Log
    private double getBlue() {
        return colorSensor.getColor().blue;
    }

    @Log
    private int getIRReading() {
        return colorSensor.getIR();
    }

    @Log
    private double getProximity() {
        return colorSensor.getProximity();
    }

    private ColorMatchResult getColorMatchResult() {
        return colorMatcher.matchClosestColor(colorSensor.getColor());
    }

    @Log
    private double getColorMatchConfidence() {
        return getColorMatchResult().confidence;
    }

    public Color getColorMatch() {
        return getColorMatchResult().color;
    }

    @Log
    public String getStringColorMatch() {
        String color;

        if (getColorMatch() == BLUE_TARGET)
            color = "Blue";
        else if (getColorMatch() == RED_TARGET)
            color = "Red";
        else if (getColorMatch() == GREEN_TARGET)
            color = "Green";
        else if (getColorMatch() == YELLOW_TARGET)
            color = "Yellow";
        else
            color = "Unknown";

        return color;
    }

    @Log
    public int getPosition() {
        return spinner.getSelectedSensorPosition();
    }

    public boolean hasSpunWheel() {
        return spinner.getClosedLoopError() <= ALLOWED_POSITION_DEVIATION;
    }

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
