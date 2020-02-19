package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team175.robot.utils.DriveHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Drive represents the drivetrain of the robot. It is composed of 4 cim motors (controlled with 4 Talon SRXs) and a
 * Pigeon gyro. This class is packed with documentation to better understand design choices and robot programming in
 * general.
 */
public final class Drive extends SubsystemBase {

    private final TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
    private final PigeonIMU gyro;
    private final DoubleSolenoid shifter;
    private final DriveHelper driveHelper;
    private final DifferentialDriveOdometry odometer;

    private static final int PCM_PORT = 17;
    private static final int LEFT_MASTER_PORT = 2;
    private static final int LEFT_SLAVE_PORT = 1;
    private static final int RIGHT_MASTER_PORT = 15;
    private static final int RIGHT_SLAVE_PORT = 14;
    private static final int SHIFTER_FORWARD_CHANNEL = 0;
    private static final int SHIFTER_REVERSE_CHANNEL = 1;

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
        gyro = new PigeonIMU(leftSlave);
        shifter = new DoubleSolenoid(PCM_PORT, SHIFTER_FORWARD_CHANNEL, SHIFTER_REVERSE_CHANNEL);
        configureTalons();
        configurePigeon();
        driveHelper = new DriveHelper(leftMaster, rightMaster);
        odometer = new DifferentialDriveOdometry(getHeading());
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

    private void configurePigeon() {
        gyro.configFactoryDefault();
        gyro.setFusedHeading(0);
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

    public void shift(boolean shift) {
        shifter.set(shift ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    @Log
    public double getLeftDemand() {
        return leftMaster.getMotorOutputPercent();
    }

    @Log
    public double getLeftVoltage() {
        return leftMaster.getMotorOutputVoltage();
    }

    /*@Log
    public int getLeftVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }*/

    @Log
    public double getRightDemand() {
        return rightMaster.getMotorOutputPercent();
    }

    @Log
    public double getRightVoltage() {
        return rightMaster.getMotorOutputVoltage();
    }

    /*@Log
    public int getRightVelocity() {
        return rightMaster.getSelectedSensorVelocity();
    }*/

    @Log.ToString
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getFusedHeading(), 360));
    }

    @Log
    public boolean isInHighGear() {
        return shifter.get() == DoubleSolenoid.Value.kForward;
    }

    @Log.ToString
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    @Override
    public void periodic() {
        // TODO: Fix and add encoders
        odometer.update(getHeading(), 0, 0);
    }

    @Override
    public void resetSensors() {
        gyro.setFusedHeading(0);
        odometer.resetPosition(new Pose2d(), getHeading());
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}