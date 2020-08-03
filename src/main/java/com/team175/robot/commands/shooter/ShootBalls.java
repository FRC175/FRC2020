package com.team175.robot.commands.shooter;

import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class ShootBalls extends CommandBase {

    private final Shooter shooter;
    private final Intake intake;

    public ShootBalls(Shooter shooter, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;
    }

    @Override
    public void initialize() {
        shooter.setBallGate(true);
        intake.setIndexerOpenLoop(1);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.setHoodHeading(Rotation2d.fromDegrees(0));
        shooter.setFlywheelOpenLoop(0);
        intake.setIndexerOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
