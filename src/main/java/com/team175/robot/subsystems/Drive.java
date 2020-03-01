package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.team175.robot.Robot;
import com.team175.robot.utils.TalonSRXDiagnostics;
import com.team175.robot.utils.DriveHelper;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Drive represents the drivetrain. It is composed of 4 CIM motors (all controlled with Talon SRXs), a Pigeon gyro, and
 * two pneumatic pistons. This class is packed with documentation to better understand design choices and robot
 * programming in general.
 */
public final class Drive extends SubsystemBase {

    // These variables are final because they only need to be instantiated once (after all, you don't need to create a
    // new left master TalonSRX).
    private final TalonSRX leftMaster, leftSlave, rightMaster, rightSlave;
    private final PigeonIMU gyro;
    private final DoubleSolenoid shifter;
    private final DriveHelper driveHelper;
    private final DifferentialDriveOdometry odometer;

    private static final int PCM_PORT = 18;
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
        configureTalons();
        gyro = new PigeonIMU(rightSlave);
        // gyro = new PigeonIMU(leftSlave);
        configurePigeon();
        shifter = new DoubleSolenoid(PCM_PORT, SHIFTER_FORWARD_CHANNEL, SHIFTER_REVERSE_CHANNEL);
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
        leftMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        leftMaster.setSelectedSensorPosition(0);
        leftMaster.setSensorPhase(true);

        leftSlave.configFactoryDefault();
        leftSlave.follow(leftMaster);
        leftSlave.setInverted(InvertType.FollowMaster);

        rightMaster.configFactoryDefault();
        rightMaster.setInverted(true);
        rightMaster.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        rightMaster.setSelectedSensorPosition(0);

        rightSlave.configFactoryDefault();
        rightSlave.follow(rightMaster);
        rightSlave.setInverted(InvertType.FollowMaster);
    }

    /**
     * Helper method that configures the Pigeon gyro.
     */
    private void configurePigeon() {
        gyro.configFactoryDefault();
        gyro.setFusedHeading(0);
    }

    /**
     * Sets the drive motors to a certain percent output (demand) using open loop control (no sensors in feedback loop).
     *
     * @param leftDemand The percent output for the left drive motors
     * @param rightDemand The percent output for the right drive motors
     */
    public void setOpenLoop(double leftDemand, double rightDemand) {
        leftMaster.set(ControlMode.PercentOutput, leftDemand);
        rightMaster.set(ControlMode.PercentOutput, rightDemand);
    }

    /**
     * Controls the drive motor using arcade controls - with a throttle and a turn.
     *
     * @param throttle The throttle from the controller
     * @param turn The turn from the controller
     */
    public void arcadeDrive(double throttle, double turn) {
        driveHelper.arcadeDrive(throttle, turn);
    }

    /**
     * "Cheesy Drive" simply means that the "turning" stick controls the curvature of the robot's path rather than its
     * rate of heading change. This helps make the robot more controllable at high speeds. It also handles the robot's
     * quick turn functionality - "quick turn" overrides constant-curvature turning for turn-in-place maneuvers.
     *
     * @param throttle The throttle from the controller
     * @param turn The turn from the controller
     * @param isQuickTurn Allow the robot to turn in place
     */
    public void cheesyDrive(double throttle, double turn, boolean isQuickTurn) {
        driveHelper.cheesyDrive(throttle, turn, isQuickTurn, true);
    }

    /**
     * Shifts the robot between the two gears - high gear and low gear.
     *
     * @param shift Shift the robot to high gear
     */
    public void shift(boolean shift) {
        shifter.set(shift ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
    }

    /**
     * Returns the current output as a percent of the left drive motors.
     *
     * @return Left motors' percent output
     */
    // The @Log is used in order to display this value on the Shuffleboard for testing
    @Log
    public double getLeftDemand() {
        return leftMaster.getMotorOutputPercent();
    }

    @Log
    public double getLeftVoltage() {
        return leftMaster.getMotorOutputVoltage();
    }

    @Log
    public int getLeftPosition() {
        return leftMaster.getSelectedSensorPosition();
    }

    @Log
    public int getLeftVelocity() {
        return leftMaster.getSelectedSensorVelocity();
    }

    @Log
    public double getRightDemand() {
        return rightMaster.getMotorOutputPercent();
    }

    @Log
    public double getRightVoltage() {
        return rightMaster.getMotorOutputVoltage();
    }

    @Log
    public int getRightPosition() {
        return rightMaster.getSelectedSensorPosition();
    }

    @Log
    public int getRightVelocity() {
        return rightMaster.getSelectedSensorVelocity();
    }

    /**
     * Returns the heading (angle) of the robot relative to its starting heading. It is returned as a {@link Rotation2d}
     * because it is specifically designed to handle angles (rotations).
     *
     * @return Current heading
     */
    // methodName gets the output of a specific method in Rotation2d
    // This is used when the return type (i.e. Rotation2d) is not natively supported by the Shuffleboard
    @Log(methodName = "getDegrees")
    public Rotation2d getHeading() {
        return Rotation2d.fromDegrees(Math.IEEEremainder(gyro.getFusedHeading(), 360));
    }

    // @Log
    public boolean isInHighGear() {
        return shifter.get() == DoubleSolenoid.Value.kForward;
    }

    /**
     * Returns the pose (position and orientation) of the robot relative to its starting pose. This is calculated by
     * the odometer, which uses forward kinematics to fuse the encoder readings from the the motors and the gyro
     * reading.
     *
     * @return Current heading
     */
    @Log.ToString
    public Pose2d getPose() {
        return odometer.getPoseMeters();
    }

    /**
     * Performs actions to run periodically (à la {@link Robot::teleopPeriodic()}). Feeds encoder and gyro readings to
     * the odometer to update the pose of the robot relative to the field.
     */
    @Override
    public void periodic() {
        // TODO: Fix and add encoders
        odometer.update(getHeading(), 0, 0);
    }

    /**
     * Resets all the sensors on the drivetrain including the encoders, gyro, and the odometer.
     */
    @Override
    public void resetSensors() {
        gyro.setFusedHeading(0);
        odometer.resetPosition(new Pose2d(), getHeading());
    }

    /**
     * Runs a self-test on the motors of the drivetrain to ensure that they are performing properly.
     */
    @Override
    public boolean checkIntegrity() {
        TalonSRXDiagnostics leftMotorTest = new TalonSRXDiagnostics(leftMaster, "DriveLeftMaster");
        TalonSRXDiagnostics rightMotorTest = new TalonSRXDiagnostics(rightMaster, "DriveRightMaster");

        logger.info("Beginning integrity check for Drive.");

        boolean isGood = true;
        isGood &= leftMotorTest.runIntegrityTest();
        isGood &= rightMotorTest.runIntegrityTest();
        logger.info("{}", leftMotorTest.toString());
        logger.info("{}", rightMotorTest.toString());

        return isGood;
    }

}