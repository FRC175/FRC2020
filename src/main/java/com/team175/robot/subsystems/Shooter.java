package com.team175.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.team175.robot.models.Gains;
import com.team175.robot.positions.TurretCardinal;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Shooter extends SubsystemBase {

    private final TalonSRX turret;

    private int turretSetpoint;

    private static final int TURRET_PORT = 5;
    private static final int TURRET_DEADBAND = 5;
    private static final Gains TURRET_GAINS = new Gains(0, 0, 0, 0, 0, 0);

    private static Shooter instance;

    private Shooter() {
        turret = new TalonSRX(TURRET_PORT);

        configureTalons();
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
        turret.config_kP(0, TURRET_GAINS.getKp());
        turret.config_kI(0, TURRET_GAINS.getKi());
        turret.config_kD(0, TURRET_GAINS.getKd());
        turret.config_kF(0, TURRET_GAINS.getKf());
        turret.configMotionAcceleration(TURRET_GAINS.getAcceleration());
        turret.configMotionCruiseVelocity(TURRET_GAINS.getCruiseVelocity());

        // Homing
        // setTurretAngle(0);
    }

    public void setTurretOpenLoop(double demand) {
        turret.set(ControlMode.PercentOutput, demand);
    }

    private void setTurretPosition(int position) {
        turretSetpoint = position;
        turret.set(ControlMode.MotionMagic, turretSetpoint);
    }

    public void setTurretAngle(int angle) {
        // TODO: Make this actually work
        setTurretPosition(angle);
    }

    public void setTurretCardinal(TurretCardinal cardinal) {
        setTurretAngle(cardinal.toDegrees());
    }

    public int getTurretPosition() {
        return turret.getSelectedSensorPosition();
    }

    public int getTurretAngle() {
        // TODO: Make this work
        return turret.getSelectedSensorPosition();
    }

    /*@Override
    public void periodic() {
        SmartDashboard.putNumber("Turret Position", getTurretPosition());
        SmartDashboard.putNumber("Turret Setpoint", turretSetpoint);
    }*/

    @Override
    public void resetSensors() {
    }

    @Override
    public boolean checkIntegrity() {
        return false;
    }

}
