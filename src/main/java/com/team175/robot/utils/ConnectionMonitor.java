package com.team175.robot.utils;

import com.team175.robot.subsystems.LED;
import com.team175.robot.subsystems.Limelight;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * ConnectionMonitor keeps track of the robot's connection to the driver station. If the robot disconnects for more than
 * 1 second, the LEDs will start blinking.
 *
 * @author Team195
 */
public class ConnectionMonitor {

    private final LED led;
    private final LatchedBoolean justReconnected;
    private final LatchedBoolean justDisconnected;
    private final Logger logger;
    private final Notifier notifier;
    private final Runnable connectionRunnable = new Runnable() {
        @Override
        public void run() {
            try {
                if (isFirstRun) {
                    Thread.currentThread().setName("ConnectionMonitor");
                    // Thread.currentThread().setPriority(Constants.kConnectionMonitorThreadPriority);
                    lastPacketTime = Timer.getFPGATimestamp();
                    isFirstRun = false;
                }

                hasConnection = DriverStation.getInstance().waitForData(1);

                if (hasConnection) {
                    lastPacketTime = Timer.getFPGATimestamp();
                } else {
                    /*mLED.setLEDColor(Constants.kCommLossColor);
                    mLED.setMessage("sos", true);*/
                    // Limelight.getInstance().turnOnLED();
                }

                if (justReconnected.update(hasConnection)) {
                    justReconnected();
                }

                if (justDisconnected.update(!hasConnection)) {
                    justDisconnected();
                }
            } catch (Throwable t) {
                logger.error("Connection Monitor has thrown an error!!!", t);
            }
        }
    };

    private boolean hasConnection;
    private double lastPacketTime;
    private boolean isFirstRun;

    private static final double NOTIFIER_PERIOD = 0.5; // s

    private static ConnectionMonitor instance;

    private ConnectionMonitor() {
        led = LED.getInstance();
        justReconnected = new LatchedBoolean();
        justDisconnected = new LatchedBoolean();
        logger = LoggerFactory.getLogger(getClass().getSimpleName());
        hasConnection = true;
        isFirstRun = true;

        notifier = new Notifier(connectionRunnable);
        notifier.startPeriodic(NOTIFIER_PERIOD);
    }

    public static ConnectionMonitor getInstance() {
        if (instance == null) {
            instance = new ConnectionMonitor();
        }

        return instance;
    }

    /*public boolean isConnected() {
        return hasConnection;
    }

    public double getLastPacketTime() {
        return lastPacketTime;
    }*/

    private void justReconnected() {
        // Reconfigure blink if we are just connected.
        /*mLED.setLEDColor(Constants.kDefaultColor);
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration);
        mLED.setRequestedState(LEDController.LEDState.BLINK);*/
        // Limelight.getInstance().turnOffLED();
    }

    private void justDisconnected() {
        // Reconfigure blink if we are just disconnected.
        // Limelight.getInstance().turnOnLED();
        // mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration * 2.0);
    }

}