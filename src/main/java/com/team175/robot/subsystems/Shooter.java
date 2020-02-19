package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ControlType;
import com.team175.robot.models.MotionMagicGains;
import com.team175.robot.positions.TurretCardinal;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Solenoid;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

/**
 * Shooter represents the shooting mechanism and turret. It is composed of 3 775-Pro motors (all controlled by Talon
 * SRX motor controllers) (two for the flywheel and one for the turret), 1 servo, and 1 pneumatic piston.
 */
public final class Shooter extends SubsystemBase {

    private final TalonSRX turret, shooterMaster, shooterSlave;
    private final CANSparkMax hood;
    private final Solenoid ballGate;
    private final MotionMagicGains turretGains;

    private int turretSetpoint;
    private int shooterSetpoint;
    private int hoodSetpoint;

    private static final int PCM_PORT = 17;
    private static final int TURRET_PORT = 11;
    private static final int SHOOTER_MASTER_PORT = 13;
    private static final int SHOOTER_SLAVE_PORT = 12;
    private static final int HOOD_PORT = 4;
    private static final int BALL_GATE_CHANNEL = 6;
    private static final int TURRET_DEADBAND = 5;
    private static final int COUNTS_PER_REVOLUTION = 4096; // TODO: Fix

    private static Shooter instance;

    private Shooter() {
        turret = new TalonSRX(TURRET_PORT);
        shooterMaster = new TalonSRX(SHOOTER_MASTER_PORT);
        shooterSlave = new TalonSRX(SHOOTER_SLAVE_PORT);
        hood = new CANSparkMax(HOOD_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
        ballGate = new Solenoid(PCM_PORT, BALL_GATE_CHANNEL);
        configureTalons();
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
        // turret.configAllowableClosedloopError(0, degreesToCounts(TURRET_DEADBAND));
        shooterMaster.configFactoryDefault();
        shooterSlave.configFactoryDefault();
        shooterSlave.follow(shooterMaster);

        // TODO: Homing position
        // setTurretAngle(0);
    }

    private void configureSparkMax() {
        hood.restoreFactoryDefaults();

        // TODO: Homing position
    }

    private int degreesToCounts(Rotation2d heading) {
        return (int) (heading.getDegrees() * (COUNTS_PER_REVOLUTION / 360.0));
    }

    private Rotation2d countsToDegrees(double position) {
        return Rotation2d.fromDegrees(position * (360.0 / COUNTS_PER_REVOLUTION));
    }

    /**
     * @return Velocity in sensor units per 100 ms
     */
    private int rpmToTalonVelocity(int rpm) {
        return rpm;
    }

    private int talonVelocityToRPM(int velocity) {
        return velocity;
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
        setTurretPosition(degreesToCounts(heading));
    }

    @Config
    private void setLoggableTurretHeading(int heading) {
        setTurretHeading(Rotation2d.fromDegrees(heading));
    }

    public void setTurretCardinal(TurretCardinal cardinal) {
        setTurretHeading(cardinal.toRotation2d());
    }

    public void setShooterOpenLoop(double demand) {
        shooterMaster.set(ControlMode.PercentOutput, demand);
    }

    public void setHoodOpenLoop(double demand) {
        hood.set(demand);
    }

    public void setHoodPosition(double position) {
        hood.getPIDController().setReference(position, ControlType.kPosition);
    }

    public void setBallGate(boolean allowBalls) {
        ballGate.set(allowBalls);
    }

    private int getTurretSetpoint() {
        return turretSetpoint;
    }

    @Log
    public int getTurretPosition() {
        return turret.getSelectedSensorPosition();
    }

    @Log.ToString
    public Rotation2d getTurretHeading() {
        return countsToDegrees(getTurretPosition());
    }

    @Log
    private double getShooterDemand() {
        return shooterMaster.getMotorOutputPercent();
    }

    @Log
    public boolean getBallGate() {
        return ballGate.get();
    }

    @Override
    public void resetSensors() {
        turret.setSelectedSensorPosition(0);
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
