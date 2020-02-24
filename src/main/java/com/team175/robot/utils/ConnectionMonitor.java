package com.team175.robot.utils;

/*import com.team195.frc2019.constants.Constants;
import com.team195.frc2019.reporters.ConsoleReporter;
import com.team195.frc2019.reporters.MessageLevel;
import com.team195.frc2019.controllers.LEDController;
import com.team195.lib.util.ThreadRateControl;
import com.team254.lib.util.CrashTrackingRunnable;
import com.team254.lib.util.LatchedBoolean;*/
import com.team175.robot.subsystems.LED;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.Timer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

/**
 * Keeps track of the robot's connection to the driver station. If it disconnects for more than 1 second, start blinking
 * the LEDs.
 */
public class ConnectionMonitor {

    private static final double MIN_CONNECTION_MONITOR_THREAD_LOOP_MS = 0.5;
    private static ConnectionMonitor instance = new ConnectionMonitor();
    private boolean hasConnection = true;
    private double mLastPacketTime;
    private LatchedBoolean mJustReconnected = new LatchedBoolean();
    private LatchedBoolean mJustDisconnected = new LatchedBoolean();
    private LED mLED = LED.getInstance();
    private Logger logger = LoggerFactory.getLogger(getClass().getSimpleName());

    private boolean isFirstRun = true;

    private final Runnable mConnectionRunnable = new Runnable() {
        @Override
        public void run() {
            try {
                if (isFirstRun) {
                    Thread.currentThread().setName("ConnectionMonitor");
                    // Thread.currentThread().setPriority(Constants.kConnectionMonitorThreadPriority);
                    mLastPacketTime = Timer.getFPGATimestamp();
                    isFirstRun = false;
                }

                hasConnection = DriverStation.getInstance().waitForData(1);

                if (hasConnection) {
                    mLastPacketTime = Timer.getFPGATimestamp();
                } else {
                    /*mLED.setLEDColor(Constants.kCommLossColor);
                    mLED.setMessage("sos", true);*/
                }

                if (mJustReconnected.update(hasConnection))
                    justReconnected();

                if (mJustDisconnected.update(!hasConnection))
                    justDisconnected();
            } catch (Throwable t) {
                logger.error("Connection Monitor has an error!!!", t);
            }
        }
    };

    private final Notifier mConnectionNotifier;

    private ConnectionMonitor() {
        mConnectionNotifier = new Notifier(mConnectionRunnable);
        mConnectionNotifier.startPeriodic(MIN_CONNECTION_MONITOR_THREAD_LOOP_MS);
    }

    public static ConnectionMonitor getInstance() {
        return instance;
    }

    public boolean isConnected() {
        return hasConnection;
    }

    public double getLastPacketTime() {
        return mLastPacketTime;
    }

    private void justReconnected() {
        // Reconfigure blink if we are just connected.
        /*mLED.setLEDColor(Constants.kDefaultColor);
        mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration);
        mLED.setRequestedState(LEDController.LEDState.BLINK);*/
    }

    private void justDisconnected() {
        // Reconfigure blink if we are just disconnected.
        // mLED.configureBlink(LEDController.kDefaultBlinkCount, LEDController.kDefaultBlinkDuration * 2.0);
    }

}