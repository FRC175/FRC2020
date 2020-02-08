package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import io.github.oblarg.oblog.annotations.Log;

public final class ColorWheelSpinner extends SubsystemBase {

    private final TalonSRX spinner;
    private final DoubleSolenoid deployer;

    private static final int PCM_PORT = 17;
    private static final int SPINNER_PORT = 5;
    private static final int DEPLOYER_FORWARD_CHANNEL = 2;
    private static final int DEPLOYER_REVERSE_CHANNEL = 3;

    private final I2C.Port i2cPort = I2C.Port.kOnboard;
    private final ColorSensorV3 colorSensor;
    private final ColorMatch colorMatcher;

    private static final Color BLUE_TARGET = ColorMatch.makeColor(0.143, 0.427, 0.429);
    private static final Color GREEN_TARGET = ColorMatch.makeColor(0.197, 0.561, 0.240);
    private static final Color RED_TARGET = ColorMatch.makeColor(0.561, 0.232, 0.114);
    private static final Color YELLOW_TARGET = ColorMatch.makeColor(0.361, 0.524, 0.113);

    private static ColorWheelSpinner instance;

    private ColorWheelSpinner() {
        spinner = new TalonSRX(SPINNER_PORT);
        deployer = new DoubleSolenoid(PCM_PORT, DEPLOYER_FORWARD_CHANNEL, DEPLOYER_REVERSE_CHANNEL);
        colorSensor = new ColorSensorV3(i2cPort);
        colorMatcher = new ColorMatch();
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

        Color detectedColor = colorSensor.getColor();

        /**
         * The sensor returns a raw IR value of the infrared light detected.
         */
        double IR = colorSensor.getIR();

        String color;
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);

        if (match.color == BLUE_TARGET)
            color = "Blue";
        else if (match.color == RED_TARGET)
            color = "Red";
        else if (match.color == GREEN_TARGET)
            color = "Green";
        else if (match.color == YELLOW_TARGET)
            color = "Yellow";
        else
            color = "Unknown";


        /**
         * Open Smart Dashboard or Shuffleboard to see the color detected by the
         * sensor.
         */
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
        SmartDashboard.putNumber("IR", IR);
        SmartDashboard.putNumber("Confidence", match.confidence);
        SmartDashboard.putString("Detected Color", color);

        /**
         * In addition to RGB IR values, the color sensor can also return an
         * infrared proximity value. The chip contains an IR led which will emit
         * IR pulses and measure the intensity of the return. When an object is
         * close the value of the proximity will be large (max 2047 with default
         * settings) and will approach zero when the object is far away.
         *
         * Proximity can be used to roughly approximate the distance of an object
         * or provide a threshold for when an object is close enough to provide
         * accurate color values.
         */
        int proximity = colorSensor.getProximity();

        SmartDashboard.putNumber("Proximity", proximity);
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
