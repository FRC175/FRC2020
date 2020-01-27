package com.team175.robot.commands;

import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;

public final class RotateTurretToTarget extends CommandBase {

    private final Shooter shooter;
    private final Limelight limelight;

    public RotateTurretToTarget(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
        addRequirements(this.shooter, this.limelight);
    }

    @Override
    public void initialize() {
        // limelight.setLED(true);
        shooter.setTurretOpenLoop(0);
    }

    @Override
    public void execute() {
        limelight.calculateTargetDrive();
        shooter.setTurretOpenLoop(limelight.getRotation());
    }

    @Override
    public void end(boolean interrupted) {
        // limelight.setCameraMode(false);
    }

    @Override
    public boolean isFinished() {
        return limelight.isAtTarget();
    }

}

