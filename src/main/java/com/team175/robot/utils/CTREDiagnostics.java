package com.team175.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * CTREDiagnostics runs a series of diagnostics tests on a CTRE motor controller to verify motor integrity.
 */
public final class CTREDiagnostics {

    private final BaseMotorController motorController;
    private final String name;

    private boolean isOutputGood, isEncoderPresent, isEncoderInPhase, isNotFaulted;

    private static final Logger logger = LoggerFactory.getLogger(CTREDiagnostics.class.getSimpleName());
    private static final int TIMEOUT = 10; // ms
    private static final double DELAY_TIME = 1.5; // Seconds

    public CTREDiagnostics(BaseMotorController motorController, String name) {
        this.motorController = motorController;
        this.name = name;
    }

    public static void checkCommand(ErrorCode code, String msg) {
        if (code != ErrorCode.OK) {
            logger.warn("{}\n" +
                    "\tErrorCode: {}", msg, code.toString());
            // LED.getInstance().blinkColor(LEDColor.ERROR);
        }
    }

    /*private boolean checkOutputDirection() {
        double prevPower = motorController.getMotorOutputPercent();
        motorController.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(DELAY_TIME);

        double power = motorController.getMotorOutputPercent();
        motorController.set(ControlMode.PercentOutput, 0);
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

    private boolean checkEncoderPresence() {
        if (motorController instanceof TalonSRX) {
            if (((TalonSRX) motorController).getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
                logger.warn("{} does not have an encoder attached!", name);
                return false;
            } else {
                logger.info("{} passed encoder existence test.", name);
                return true;
            }
        } else {
            logger.warn("{} does not support encoder existence testing!", name);
            return false; // Return true in order to still run sensor phase test
        }
    }

    private boolean checkEncoderPhase() {
        // Move motor for 2 seconds
        motorController.setSelectedSensorPosition(0);
        double preTestPosition = motorController.getSelectedSensorPosition();
        motorController.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(DELAY_TIME);

        // Wait 2 seconds before stopping motor
        double postTestPosition = motorController.getSelectedSensorPosition();
        Timer.delay(DELAY_TIME);
        motorController.set(ControlMode.PercentOutput, 0);

        if (postTestPosition <= preTestPosition) {
            logger.warn("{} encoder out of phase!", name);
            return false;
        } else {
            logger.info("{} passed encoder phase test.", name);
            return true;
        }
    }

    private boolean checkFaults() {
        if (motorController.hasResetOccurred()) {
            if (motorController.clearStickyFaults(TIMEOUT) != ErrorCode.OK) {
                logger.warn("{} failed to clear sticky faults!", name);
                return false;
            }
        }

        logger.info("{} passed faults test!", name);
        return true;
    }

    public boolean checkMotorController() {
        logger.info("Beginning diagnostics test for {}.", name);

        // isOutputGood = true;
        isEncoderPresent = checkEncoderPresence();
        // isEncoderInPhase = isEncoderPresent ? checkEncoderPhase() : false;
        isEncoderInPhase = checkEncoderPhase();
        isNotFaulted = checkFaults(); // Invert to see if motor is faulted

        boolean isGood = true;
        // isGood &= isOutputGood;
        isGood &= isEncoderPresent;
        isGood &= isEncoderInPhase;
        isGood &= isNotFaulted; // Invert again to see if motor is good

        if (!isGood) {
            logger.error("{} failed diagnostics test!", name);
        }

        return isGood;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Diagnostics Test Summary for ").append(name).append(":\n");
        // sb.append("Output Direction: ").append(isOutputGood).append("\n");
        sb.append("Encoder Existence: ").append(isEncoderPresent).append("\n");
        sb.append("Sensor Phase: ").append(isEncoderInPhase).append("\n");
        sb.append("Motor Faulted?: ").append(!isNotFaulted);
        return sb.toString();
    }

}