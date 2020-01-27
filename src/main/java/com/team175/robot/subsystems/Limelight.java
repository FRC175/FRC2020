package com.team175.robot.subsystems;

import com.team175.robot.models.Gains;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.controller.PIDController;

/**
 * Limelight represents the limelight vision processing unit on the robot.
 */
public final class Limelight extends SubsystemBase {

    private final NetworkTable table;
    private final PIDController rotationController;

    private double rotation;
    private boolean isAtTarget;

    public static final int ROTATION_SETPOINT = 0; // Target at the center of the limelight
    private static final int ROTATION_DEADBAND = 2; // Degrees
    private static final Gains ROTATION_GAINS = new Gains(-0.05, 0, 0);

    private static Limelight instance;

    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        rotationController = new PIDController(ROTATION_GAINS.getKp(), ROTATION_GAINS.getKi(), ROTATION_GAINS.getKd());

        rotationController.setTolerance(ROTATION_DEADBAND);
        // The range of the limelight is between -30 degrees and 30 degrees
        rotationController.enableContinuousInput(-30, 30);

        configureTelemetry();
    }

    public static Limelight getInstance() {
        if (instance == null) {
            instance = new Limelight();
        }

        return instance;
    }

    private void configureTelemetry() {
        telemetry.put("IsTargetDetected", this::isTargetDetected);
        telemetry.put("HorizontalOffset", this::getHorizontalOffset);
        telemetry.put("Rotation", this::getRotation);
        telemetry.put("PipelineNum", this::getPipeline);
        telemetry.put("CalculatedRotation", this::getRotation);
    }

    private void setPipeline(int pipelineNum) {
        table.getEntry("pipeline").setNumber(pipelineNum);
    }

    private boolean isTargetDetected() {
        return table.getEntry("tv").getDouble(0) == 1;
    }

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

    private int getPipeline() {
        return (int) table.getEntry("getpipe").getDouble(0);
    }

    public void setLED(boolean enable) {
        table.getEntry("ledMode").setNumber(enable ? 3 : 1);
    }

    public void blinkLED() {
        table.getEntry("ledMode").setNumber(2);
    }

    public void defaultLED() {
        table.getEntry("ledMode").setNumber(0);
    }

    public void setCameraMode(boolean isTrackingMode) {
        // 0 => Tracking Mode; 1 => Driver Mode
        setPipeline(isTrackingMode ? 0 : 1);
        // table.getEntry("camMode").setNumber(isTrackingMode ? 0 : 1);
    }

    public double getRotation() {
        return rotation;
    }

    public boolean isAtTarget() {
        return isAtTarget;
    }

    public void calculateTargetDrive() {
        if (isTargetDetected()) {
            // Proportional turn based on tx
            rotation = rotationController.calculate(getHorizontalOffset(), 0); // TODO: Make constant

            isAtTarget = rotationController.atSetpoint();

            logger.debug("Turn = {}", rotation);
            logger.debug("IsAtTarget = {}", isAtTarget);
        } else {
            // turn = SEEK_TURN;
            rotation = 0;
            // isAtTarget = false;
            isAtTarget = true;
            logger.warn("NO TARGET DETECTED!!! TURN THE TURRET TO A CARDINAL SO THAT IT SEES THE TARGET.");
        }
    }

    @Override
    public void resetSensors() {
        setPipeline(0);
        defaultLED();
        setCameraMode(false);
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}