package com.team175.robot.commands;

import com.team175.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ControlTurret extends CommandBase {

    private final Shooter shooter;
    private final DoubleSupplier demand;

    public ControlTurret(Shooter shooter, DoubleSupplier demand) {
        this.shooter = shooter;
        this.demand = demand;
        addRequirements(shooter);
    }

    @Override
    public void execute() {
        shooter.setTurretOpenLoop(demand.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setTurretOpenLoop(0);
    }

}
