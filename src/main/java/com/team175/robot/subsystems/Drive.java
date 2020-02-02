package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team175.robot.utils.DriveHelper;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Drive represents the drivetrain of the robot. It is composed of 4 cim motors (controlled with 4 Talon SRXs) and a
 * Pigeon gyro. This class is packed with documentation to better understand design choices and robot programming in
 * general.
 */
public class Drive extends SubsystemBase {

    private final TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
    private final DriveHelper driveHelper;
    // private final DoubleSolenoid shifter;

    private static final int LEFT_MASTER_PORT = 1;
    private static final int LEFT_SLAVE_PORT = 2;
    private static final int RIGHT_MASTER_PORT = 3;
    private static final int RIGHT_SLAVE_PORT = 4;
    // private static final int SHIFTER_FORWARD_CHANNEL = 0;
    // private static final int SHIFTER_REVERSE_CHANNEL = 0;

    /**
     * The single instance of {@link Drive} used to implement the "singleton" design pattern. A description of the
     * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
     */
    private static Drive instance;

    /**
     * The constructor, which is private in order to implement the "singleton" design pattern. A description of the
     * singleton design pattern can be found in the JavaDoc for {@link Drive::getInstance()}.
     */
    private Drive() {
        leftMaster = new TalonSRX(LEFT_MASTER_PORT);
        leftSlave = new TalonSRX(LEFT_SLAVE_PORT);
        rightMaster = new TalonSRX(RIGHT_MASTER_PORT);
        rightSlave = new TalonSRX(RIGHT_SLAVE_PORT);

        driveHelper = new DriveHelper(leftMaster, rightMaster);

        /*shifter = new DoubleSolenoid(SHIFTER_FORWARD_CHANNEL, SHIFTER_REVERSE_CHANNEL);*/

        configureTalons();
    }

    /**
     * <code>getInstance()</code> is a crucial part of the "singleton" design pattern. This pattern is used when there
     * should only be one instance of a class, which makes sense for Robot subsystems (after all, there is only one
     * drivetrain). The singleton pattern is implemented by making the constructor private, creating a static variable
     * of the type of the object called <code>instance</code>, and creating this method, <code>getInstance()</code>, to
     * return the instance when the class needs to be used.
     *
     * Usage:
     * <code>Drive drive = Drive.getInstance();</code>
     *
     * @return The single instance of {@link Drive}
     */
    public static Drive getInstance() {
        if (instance == null) {
            instance = new Drive();
        }

        return instance;
    }

    /**
     * Helper method that configures the Talon SRX motor controllers.
     */
    private void configureTalons() {
        leftMaster.configFactoryDefault();
        leftMaster.setInverted(false);

        leftSlave.configFactoryDefault();
        leftSlave.follow(leftMaster);
        leftSlave.setInverted(InvertType.FollowMaster);

        rightMaster.configFactoryDefault();
        rightMaster.setInverted(true);

        rightSlave.configFactoryDefault();
        rightSlave.follow(rightMaster);
        rightSlave.setInverted(InvertType.FollowMaster);
    }

    public void setOpenLoop(double leftDemand, double rightDemand) {
        leftMaster.set(ControlMode.PercentOutput, leftDemand);
        rightMaster.set(ControlMode.PercentOutput, rightDemand);
    }

    public void arcadeDrive(double throttle, double turn) {
        driveHelper.arcadeDrive(throttle, turn);
    }

    public void cheesyDrive(double throttle, double turn, boolean isQuickTurn) {
        driveHelper.cheesyDrive(throttle, turn, isQuickTurn, true);
    }

    /*public void setHighGear(boolean shift) {
        shifter.set(shift);
    }*/

    @Log
    public double getLeftDemand() {
        return leftMaster.getMotorOutputPercent();
    }

    @Log
    public double getLeftVoltage() {
        return leftMaster.getMotorOutputVoltage();
    }

    @Log
    public double getRightDemand() {
        return rightMaster.getMotorOutputPercent();
    }

    @Log
    public double getRightVoltage() {
        return rightMaster.getMotorOutputVoltage();
    }

    /*public boolean isHighGear() {
        return shifter.get() == DoubleSolenoid.Value.kForward;
    }*/

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}