package com.team175.robot.subsystems;

import com.team175.robot.models.Gains;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Limelight represents the limelight vision processing unit on the robot.
 */
public final class Limelight extends SubsystemBase {

    private final NetworkTable table;
    @Log
    private final PIDController rotationController;

    private double rotation;
    private boolean isAtTarget;

    public static final int ROTATION_SETPOINT = 0; // Target at the center of the limelight
    private static final int ROTATION_DEADBAND = 2; // Degrees
    private static final Gains ROTATION_GAINS = new Gains(-0.075, 0, 0);

    private static Limelight instance;

    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        rotationController = new PIDController(ROTATION_GAINS.getKp(), ROTATION_GAINS.getKi(), ROTATION_GAINS.getKd());
        rotationController.setTolerance(ROTATION_DEADBAND);
        // The range of the limelight is between -30 degrees and 30 degrees
        rotationController.enableContinuousInput(-30, 30);
    }

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }

    @Config
    private void setPipeline(int pipelineNum) {
        table.getEntry("pipeline").setNumber(pipelineNum);
    }

    @Log
    private boolean isTargetDetected() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

    @Log
    private double getHorizontalOffset() {
        return table.getEntry("tx").getDouble(0);
    }

    private double getVerticalOffset() {
        return table.getEntry("ty").getDouble(0);
    }

    private double getTargetArea() {
        return table.getEntry("ta").getDouble(0);
    }

    private double getSkew() {
        return table.getEntry("ts").getDouble(0);
    }

    @Log
    private int getPipeline() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    public void setLED(boolean enable) {
        table.getEntry("ledMode").setNumber(enable ? 3 : 1);
    }

    public void turnOnLED() {
        setLED(true);
    }

    public void turnOffLED() {
        setLED(false);
    }

    public void blinkLED() {
        table.getEntry("ledMode").setNumber(2);
    }

    public void defaultLED() {
        table.getEntry("ledMode").setNumber(0);
    }

    public void setDriverMode() {
        setPipeline(1);
    }

    public void setTrackingMode() {
        // Zoomed tracking mode
        setPipeline(2);
    }

    @Log
    public double getRotation() {
        return rotation;
    }

    @Log
    public boolean isAtTarget() {
        return isAtTarget;
    }

    public void calculateRotation() {
        if (isTargetDetected()) {
            rotation = rotationController.calculate(getHorizontalOffset(), ROTATION_SETPOINT);
            isAtTarget = rotationController.atSetpoint();
            logger.debug("Rotation = {}", rotation);
            logger.debug("IsAtTarget = {}", isAtTarget);
        } else {
            rotation = 0;
            isAtTarget = true;
            logger.warn("NO TARGET DETECTED!!! ROTATE THE TURRET TO A CARDINAL SO THAT IT SEES THE TARGET.");
        }
    }

    @Override
    public void resetSensors() {
        setPipeline(0);
        defaultLED();
        setDriverMode();
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}