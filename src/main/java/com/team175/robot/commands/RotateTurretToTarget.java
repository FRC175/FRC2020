package com.team175.robot.commands;

import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class RotateTurretToTarget extends CommandBase {

    private final Shooter shooter;
    private final Limelight limelight;

    // Shooter and Limelight objects are is the constructor despite the single instance.
    // This is because WPILib recommends the dependency injection design pattern.
    public RotateTurretToTarget(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(this.shooter, this.limelight);
    }

    @Override
    public void initialize() {
        limelight.setTrackingMode();
        shooter.setTurretOpenLoop(0);
    }

    @Override
    public void execute() {
        limelight.calculateRotation();
        shooter.setTurretOpenLoop(limelight.getRotation());
    }

    @Override
    public void end(boolean interrupted) {
        limelight.setDriverMode();
        shooter.setTurretOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return limelight.isAtTarget();
    }

}

