package com.team175.robot.utils;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * TalonSRXDiagnostics runs a series of diagnostics tests on a Talon SRX to verify motor integrity.
 */
public final class TalonSRXDiagnostics {

    private final TalonSRX motorController;
    private final String name;

    private boolean isEncoderPresent, isEncoderInPhase, isNotFaulted;

    private static final Logger logger = LoggerFactory.getLogger(TalonSRXDiagnostics.class.getSimpleName());
    private static final int TIMEOUT = 10; // ms
    private static final int ENCODER_FAULT_THRESHOLD = 2; // counts
    private static final double TEST_DURATION = 3; // Seconds

    public TalonSRXDiagnostics(TalonSRX motorController, String name) {
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

    private boolean checkEncoderPresence() {
        if (motorController.getSensorCollection().getPulseWidthRiseToRiseUs() == 0) {
            logger.warn("{} does not have an encoder attached!", name);
            return false;
        } else {
            logger.info("{} passed encoder existence test.", name);
            return true;
        }
    }

    private boolean checkEncoderPhase() {
        // Move motor for 2 seconds
        motorController.setSelectedSensorPosition(0);
        double preTestPosition = motorController.getSelectedSensorPosition();
        motorController.set(ControlMode.PercentOutput, 0.75);
        Timer.delay(TEST_DURATION);

        // Wait 2 seconds before stopping motor
        double postTestPosition = motorController.getSelectedSensorPosition();
        Timer.delay(TEST_DURATION);
        motorController.set(ControlMode.PercentOutput, 0);

        if (Math.abs(postTestPosition - preTestPosition) <= ENCODER_FAULT_THRESHOLD) {
            logger.warn("{} encoder not connected or malfunctioning!", name);
            return false;
        } else if (postTestPosition <= preTestPosition) {
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

    public boolean runIntegrityTest() {
        logger.info("Beginning diagnostics test for {}.", name);

        isEncoderPresent = checkEncoderPresence();
        isEncoderInPhase = checkEncoderPhase();
        isNotFaulted = checkFaults();

        boolean isGood = true;
        isGood &= isEncoderPresent;
        isGood &= isEncoderInPhase;
        isGood &= isNotFaulted;

        if (!isGood) {
            logger.error("{} failed diagnostics test!", name);
        }

        return isGood;
    }

    @Override
    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Diagnostics Test Summary for ").append(name).append(":\n");
        sb.append("Encoder Existence: ").append(isEncoderPresent).append("\n");
        sb.append("Sensor Phase: ").append(isEncoderInPhase).append("\n");
        sb.append("Motor Faulted?: ").append(!isNotFaulted);
        return sb.toString();
    }

}