package com.team175.robot.utils;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * DriveHelper implements multiple different drive types (e.g., Cheesy Drive and Arcade Drive) to run on the Talon SRX.
 */
public final class DriveHelper {

    // Talon SRXs
    private TalonSRX left, right;

    // Cheesy Drive variables
    private double oldWheel = 0;
    private double quickStopAccumulator = 0;
    private double negInertiaAccumulator = 0;

    // Cheesy Drive constants that determine how fast the wheel traverses the "non linear" sine curve
    private static final double HIGH_WHEEL_NON_LINEARITY = 0.65;
    private static final double LOW_WHEEL_NON_LINEARITY = 0.5;
    private static final double HIGH_NEG_INERTIA_SCALAR = 4.0;
    private static final double LOW_NEG_INERTIA_THRESHOLD = 0.65;
    private static final double LOW_NEG_INERTIA_TURN_SCALAR = 3.5;
    private static final double LOW_NEG_INERTIA_CLOSE_SCALAR = 4.0;
    private static final double LOW_NEG_INERTIA_FAR_SCALAR = 5.0;
    private static final double HIGH_SENSITIVITY = 0.65;
    private static final double LOW_SENSITIVITY = 0.65;
    private static final double QUICK_STOP_DEADBAND = 0.5;
    private static final double QUICK_STOP_WEIGHT = 0.1;
    private static final double QUICK_STOP_SCALAR = 5.0;

    /**
     * Constructs a new FatDriveHelper.
     *
     * @param left  The master Talon SRX for the left drive motors
     * @param right The master Talon SRX for the right drive motors
     */
    public DriveHelper(TalonSRX left, TalonSRX right) {
        this.left = left;
        this.right = right;
    }

    /**
     * "Cheesy Drive" simply means that the "turning" stick controls the curvature of the robot's path rather than its
     * rate of heading change. This helps make the robot more controllable at high speeds. It also handles the robot's
     * quick turn functionality - "quick turn" overrides constant-curvature turning for turn-in-place maneuvers.
     *
     * @author Team254
     */
    public void cheesyDrive(double throttle, double turn, boolean isQuickTurn, boolean isHighGear) {
        double negInertia = turn - oldWheel;
        oldWheel = turn;

        double wheelNonLinearity;
        if (isHighGear) {
            wheelNonLinearity = HIGH_WHEEL_NON_LINEARITY;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            turn = Math.sin(Math.PI / 2.0 * wheelNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * wheelNonLinearity * turn) / denominator;
        } else {
            wheelNonLinearity = LOW_WHEEL_NON_LINEARITY;
            final double denominator = Math.sin(Math.PI / 2.0 * wheelNonLinearity);
            // Apply a sin function that's scaled to make it feel better.
            turn = Math.sin(Math.PI / 2.0 * wheelNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * wheelNonLinearity * turn) / denominator;
            turn = Math.sin(Math.PI / 2.0 * wheelNonLinearity * turn) / denominator;
        }

        double leftPwm, rightPwm, overPower;
        double sensitivity;

        double angularPower;
        double linearPower;

        // Negative inertia!
        double negInertiaScalar;
        if (isHighGear) {
            negInertiaScalar = HIGH_NEG_INERTIA_SCALAR;
            sensitivity = HIGH_SENSITIVITY;
        } else {
            if (turn * negInertia > 0) {
                // If we are moving away from 0.0, aka, trying to get more turn.
                negInertiaScalar = LOW_NEG_INERTIA_TURN_SCALAR;
            } else {
                // Otherwise, we are attempting to go back to 0.0.
                if (Math.abs(turn) > LOW_NEG_INERTIA_THRESHOLD) {
                    negInertiaScalar = LOW_NEG_INERTIA_FAR_SCALAR;
                } else {
                    negInertiaScalar = LOW_NEG_INERTIA_CLOSE_SCALAR;
                }
            }
            sensitivity = LOW_SENSITIVITY;
        }
        double negInertiaPower = negInertia * negInertiaScalar;
        negInertiaAccumulator += negInertiaPower;

        turn = turn + negInertiaAccumulator;
        if (negInertiaAccumulator > 1) {
            negInertiaAccumulator -= 1;
        } else if (negInertiaAccumulator < -1) {
            negInertiaAccumulator += 1;
        } else {
            negInertiaAccumulator = 0;
        }
        linearPower = throttle;

        // Quickturn!
        if (isQuickTurn) {
            if (Math.abs(linearPower) < QUICK_STOP_DEADBAND) {
                double alpha = QUICK_STOP_WEIGHT;
                quickStopAccumulator = (1 - alpha) * quickStopAccumulator
                        + alpha * limit(turn, 1.0) * QUICK_STOP_SCALAR;
            }
            overPower = 1.0;
            angularPower = turn;
        } else {
            overPower = 0.0;
            angularPower = Math.abs(throttle) * turn * sensitivity - quickStopAccumulator;
            if (quickStopAccumulator > 1) {
                quickStopAccumulator -= 1;
            } else if (quickStopAccumulator < -1) {
                quickStopAccumulator += 1;
            } else {
                quickStopAccumulator = 0.0;
            }
        }

        rightPwm = leftPwm = linearPower;
        leftPwm += angularPower;
        rightPwm -= angularPower;

        if (leftPwm > 1.0) {
            rightPwm -= overPower * (leftPwm - 1.0);
            leftPwm = 1.0;
        } else if (rightPwm > 1.0) {
            leftPwm -= overPower * (rightPwm - 1.0);
            rightPwm = 1.0;
        } else if (leftPwm < -1.0) {
            rightPwm += overPower * (-1.0 - leftPwm);
            leftPwm = -1.0;
        } else if (rightPwm < -1.0) {
            leftPwm += overPower * (-1.0 - rightPwm);
            rightPwm = -1.0;
        }

        left.set(ControlMode.PercentOutput, leftPwm);
        right.set(ControlMode.PercentOutput, rightPwm);
    }

    /**
     * Arcade drive using arbitrary feed forward.
     */
    public void arcadeDrive(double throttle, double turn) {
        left.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, +turn);
        right.set(ControlMode.PercentOutput, throttle, DemandType.ArbitraryFeedForward, -turn);
    }

    /**
     * The arcade drive mode from {@link edu.wpi.first.wpilibj.drive.DifferentialDrive}.
     *
     * @author Worcester Polytechnic Institute
     */
    public void worcesterArcadeDrive(double throttle, double turn) {
        // Square the inputs (while preserving the sign) to increase fine control while permitting full power.
        throttle = Math.copySign(throttle * throttle, throttle);
        turn = Math.copySign(turn * turn, turn);

        double leftMotorOutput;
        double rightMotorOutput;
        double maxInput = Math.copySign(Math.max(Math.abs(throttle), Math.abs(turn)), throttle);

        if (throttle >= 0.0) {
            // First quadrant, else second quadrant
            if (turn >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            } else {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            }
        } else {
            // Third quadrant, else fourth quadrant
            if (turn >= 0.0) {
                leftMotorOutput = throttle + turn;
                rightMotorOutput = maxInput;
            } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = throttle - turn;
            }
        }

        left.set(ControlMode.PercentOutput, limit(leftMotorOutput, 1));
        right.set(ControlMode.PercentOutput, limit(rightMotorOutput, 1));
    }

    /**
     * Forces robot to drive straight by using gyro. Must be using Pigeon gyro with tuned auxiliary PID.
     *
     * TODO: Get this to work
     */
    @Deprecated(since = "2019", forRemoval = true)
    public void straightDrive(double throttle) {
        right.set(ControlMode.PercentOutput, throttle, DemandType.AuxPID, 0); // 0 degrees => straight
        left.follow(right, FollowerType.AuxOutput1);
    }

    /**
     * From Team254's Util class.
     */
    private double limit(double v, double maxMagnitude) {
        return Math.min(maxMagnitude, Math.max(-maxMagnitude, v));
    }

}