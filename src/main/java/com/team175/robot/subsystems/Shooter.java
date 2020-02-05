package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.team175.robot.models.MotionMagicGains;
import com.team175.robot.positions.TurretCardinal;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public final class Shooter extends SubsystemBase {

    private final TalonSRX turret;
    private final VictorSPX shooterMaster;
    private final TalonSRX shooterSlave;
    @Log
    private final MotionMagicGains turretGains;

    private int turretSetpoint;

    private static final int TURRET_PORT = 11;
    private static final int SHOOTER_MASTER_PORT = 12;
    private static final int SHOOTER_SLAVE_PORT = 13;

    private static final int TURRET_DEADBAND = 5;
    private static final int COUNTS_PER_REVOLUTION = 4096; // TODO: Fix

    private static Shooter instance;

    private Shooter() {
        turret = new TalonSRX(TURRET_PORT);
        shooterMaster = new VictorSPX(SHOOTER_MASTER_PORT);
        shooterSlave = new TalonSRX(SHOOTER_SLAVE_PORT);
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
        turret.setInverted(true);
        turret.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        turret.setSensorPhase(true);
        // TODO: Comment out in real robot
        turret.setSelectedSensorPosition(0);

        shooterMaster.configFactoryDefault();

        shooterSlave.configFactoryDefault();
        shooterSlave.follow(shooterMaster);

        // Homing
        // setTurretAngle(0);
    }

    public void setOpenLoop(double demand) {
        turret.set(ControlMode.PercentOutput, demand);
        shooterMaster.set(ControlMode.PercentOutput, demand);
    }

    private int degreesToCounts(Rotation2d heading) {
        return (int) (heading.getDegrees() * (COUNTS_PER_REVOLUTION / 360.0));
    }

    private Rotation2d countsToDegrees(double position) {
        return Rotation2d.fromDegrees(position * (360.0 / COUNTS_PER_REVOLUTION));
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

    @Log
    public int getTurretPosition() {
        return turret.getSelectedSensorPosition();
    }

    public Rotation2d getTurretHeading() {
        return countsToDegrees(getTurretPosition());
    }

    @Log
    private double getLoggableTurretHeading() {
        return getTurretHeading().getDegrees();
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
