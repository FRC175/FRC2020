package com.team175.robot.commands;

import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * LockOntoTarget is essentially the same as {@link LockOntoTarget} except the command cannot end on its own (i.e. it
 * never "ends").
 */
public final class LockOntoTarget extends CommandBase {

    private final Shooter shooter;
    private final Limelight limelight;
    
    public LockOntoTarget(Shooter shooter, Limelight limelight) {
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
        return false;
    }

}
