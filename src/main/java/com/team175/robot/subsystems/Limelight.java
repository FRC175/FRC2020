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
    private final PIDController turnController;

    private double turn;
    private boolean isAtTarget;

    private static final int ROTATION_DEADBAND = 2;
    private static final Gains TURN_GAINS = new Gains(-0.05, 0, 0, 0, 0, 0);

    private static Limelight instance;

    private Limelight() {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        turnController = new PIDController(TURN_GAINS.getKp(), TURN_GAINS.getKi(), TURN_GAINS.getKd());

        // TODO: Maybe add input range
        turnController.setTolerance(ROTATION_DEADBAND);
        turnController.enableContinuousInput(-30, 30);

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
        telemetry.put("VerticalOffset", this::getVerticalOffset);
        telemetry.put("TargetArea", this::getTargetArea);
        telemetry.put("Rotation", this::getRotation);
        telemetry.put("PipelineNum", this::getPipeline);
        telemetry.put("CalculatedTurn", () -> turn);
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

    private double getRotation() {
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

    public double getTurn() {
        return turn;
    }

    public boolean isAtTarget() {
        return isAtTarget;
    }

    public void calculateTargetDrive() {
        if (isTargetDetected()) {
            // Proportional turn based on tx
            turn = turnController.calculate(getHorizontalOffset(), 0); // TODO: Make constant

            isAtTarget = turnController.atSetpoint();

            logger.debug("Turn = {}", turn);
            logger.debug("IsAtTarget = {}", isAtTarget);
        } else {
            // turn = SEEK_TURN;
            turn = 0;
            // isAtTarget = false;
            isAtTarget = true;
            logger.warn("NO TARGET DETECTED!!! Turret is turning until it sees a target.");
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