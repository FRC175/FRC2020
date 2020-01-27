package com.team175.robot.commands;

import com.team175.robot.subsystems.Shooter;

import java.util.function.DoubleSupplier;

public class ManualTurretControl extends CommandBase {

    private final Shooter shooter;
    private final DoubleSupplier demand;

    // Shooter object is the constructor (despite single instance) due to the WPILib recommending the dependency
    // injection design pattern
    public ManualTurretControl(Shooter shooter, DoubleSupplier demand) {
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
