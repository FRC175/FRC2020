package com.team175.robot.utils;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANError;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

public final class SparkMaxDiagnostics {

    private final CANSparkMax motorController;
    private final CANEncoder encoder;
    private final String name;

    private boolean isSensorInPhase;

    private static final Logger logger = LoggerFactory.getLogger(CTREDiagnostics.class.getSimpleName());
    private static final double DELAY_TIME = 2; // In seconds

    public SparkMaxDiagnostics(CANSparkMax motorController, String name) {
        this.motorController = motorController;
        this.name = name;
        encoder = motorController.getEncoder();
    }

    public static void checkCommand(CANError code, String msg) {
        if (code != CANError.kOk) {
            logger.warn("{}\n" +
                    "\tCANError: {}", msg, code.toString());
            // LED.getInstance().blinkColor(LEDColor.ERROR);
        }
    }

    /*private boolean checkOutputDirection() {
        double prevPower = motorController.getAppliedOutput();
        motorController.set(0.75);
        Timer.delay(DELAY_TIME);

        double power = motorController.getAppliedOutput();
        motorController.set(0);
        Timer.delay(DELAY_TIME);

        if (power <= prevPower) {
            logger.warn("{} must be inverted!", name);
            // return false;
            return false;
        } else {
            logger.info("{} passed output direction test.", name);
            return true;
        }
    }*/

    public boolean checkMotorController() {
        logger.info("Beginning diagnostics test for {}.", name);

        // Move motor for 2 seconds
        encoder.setPosition(0);
        double preTestPosition = encoder.getPosition();
        motorController.set(0.25);
        Timer.delay(DELAY_TIME);

        // Wait 2 seconds before stopping motor
        double postTestPosition = encoder.getPosition();
        Timer.delay(DELAY_TIME);
        motorController.set(0);

        if (postTestPosition <= preTestPosition) {
            logger.error("{} failed diagnostics test!", name);
            return isSensorInPhase = false;
        } else {
            return isSensorInPhase = true;
        }
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Diagnostics Test Summary for ").append(name).append(":\n");
        sb.append("Sensor Phase: ").append(isSensorInPhase).append("\n");
        return sb.toString();
    }

}
