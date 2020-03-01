package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.team175.robot.models.MotionMagicGains;
import com.team175.robot.positions.TurretCardinal;
import com.team175.robot.utils.SensorUnits;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Solenoid;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Shooter represents the shooting mechanism and turret. It is composed of 3 775-Pro motors (all controlled by Talon
 * SRX motor controllers) (two for the flywheel and one for the turret), 1 servo, and 1 pneumatic piston.
 */
public final class Shooter extends SubsystemBase {

    private TalonSRX turret, flywheelMaster, flywheelSlave;
    private final Servo hood;
    // private CANSparkMax hood;
    private Solenoid ballGate;
    private final MotionMagicGains turretGains;

    private int turretSetpoint;
    private int flywheelSetpoint;
    private int hoodSetpoint;

    private static final int PCM_PORT = 18;
    private static final int TURRET_PORT = 11;
    private static final int FLYWHEEL_MASTER_PORT = 13;
    private static final int FLYWHEEL_SLAVE_PORT = 12;
    private static final int HOOD_PORT = 0;
    private static final int BALL_GATE_CHANNEL = 6;
    private static final int TURRET_DEADBAND = 5;
    private static final int COUNTS_PER_REVOLUTION = 4096; // TODO: Fix

    private static Shooter instance;

    private Shooter() {
        turret = new TalonSRX(TURRET_PORT);
        flywheelMaster = new TalonSRX(FLYWHEEL_MASTER_PORT);
        flywheelSlave = new TalonSRX(FLYWHEEL_SLAVE_PORT);
        configureTalons();
        hood = new Servo(HOOD_PORT);
        // hood = new CANSparkMax(HOOD_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        configureSparkMax();
        ballGate = new Solenoid(PCM_PORT, BALL_GATE_CHANNEL);
        turretGains = new MotionMagicGains(10.1, 0, 20.2, 0, 0, 0, turret);
    }

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }

        return instance;
    }

    private void configureTalons() {
        // Configuration
        turret.configFactoryDefault();
        turret.setInverted(false);
        turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        // turret.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);
        turret.setSensorPhase(true);
        // TODO: Comment out in real robot
        turret.setSelectedSensorPosition(0);
        // TODO: Add
        // turret.configAllowableClosedloopError(0, degreesToCounts(TURRET_DEADBAND));
        flywheelMaster.configFactoryDefault();
        flywheelSlave.configFactoryDefault();
        flywheelSlave.follow(flywheelMaster);

        // TODO: Homing position
        // setTurretAngle(0);
    }

    private void configureSparkMax() {
        // hood.restoreFactoryDefaults();

        // TODO: Homing position
    }

    public void setTurretOpenLoop(double demand) {
        turret.set(ControlMode.PercentOutput, demand);
    }

    @Config
    public void setTurretPosition(int position) {
        turretSetpoint = position;
        turret.set(ControlMode.Position, turretSetpoint);
    }

    public void setTurretHeading(Rotation2d heading) {
        setTurretPosition(SensorUnits.degreesToCounts(heading, COUNTS_PER_REVOLUTION));
    }

    @Config
    private void setLoggableTurretHeading(int heading) {
        setTurretHeading(Rotation2d.fromDegrees(heading));
    }

    public void setTurretCardinal(TurretCardinal cardinal) {
        setTurretHeading(cardinal.toRotation2d());
    }

    public void setFlywheelOpenLoop(double demand) {
        flywheelMaster.set(ControlMode.PercentOutput, demand);
    }

    public void setFlywheelVelocity(int velocity) {
        flywheelMaster.set(ControlMode.Velocity, velocity);
    }

    public void setFlywheelRPM(double rpm) {
        flywheelMaster.set(ControlMode.Velocity, SensorUnits.rpmToTalonVelocity(rpm, COUNTS_PER_REVOLUTION));
    }

    public void setHoodOpenLoop(double demand) {
        hood.set(demand);
    }

    /**
     * Sets the hood position.
     *
     * @param position A number between 0 and 1 that represents the position of the servo.
     */
    public void setHoodPosition(double position) {
        hood.set(position);
    }

    public void setHoodHeading(Rotation2d heading) {
        hood.setAngle(heading.getDegrees());
    }

    public void setBallGate(boolean allowBalls) {
        logger.info("{} ball gate", allowBalls ? "Retracting" : "Deploying");
        ballGate.set(allowBalls);
    }

    private int getTurretSetpoint() {
        return turretSetpoint;
    }

    @Log
    public int getTurretPosition() {
        return turret.getSelectedSensorPosition();
    }

    @Log
    public int getTurretVelocity() {
        return turret.getSelectedSensorVelocity();
    }

    @Log(methodName = "getDegrees")
    public Rotation2d getTurretHeading() {
        return SensorUnits.countsToDegrees(getTurretPosition(), COUNTS_PER_REVOLUTION);
    }

    /*@Log
    private double getFlywheelDemand() {
        return flywheelMaster.getMotorOutputPercent();
    }

    public int getFlywheelVelocity() {
        return flywheelMaster.getSelectedSensorVelocity();
    }

    public double getFlywheelRPM() {
        return SensorUnits.talonVelocityToRPM(getFlywheelVelocity(), COUNTS_PER_REVOLUTION);
    }

    @Log
    public boolean getBallGate() {
        return ballGate.get();
    }*/

    @Override
    public void resetSensors() {
        turret.setSelectedSensorPosition(0);
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
