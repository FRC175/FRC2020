package com.team175.robot.commands.shooter;

import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class RevUpShooter extends CommandBase {

    private final Shooter shooter;
    private final Limelight limelight;

    public RevUpShooter(Shooter shooter, Limelight limelight) {
        this.shooter = shooter;
        this.limelight = limelight;
    }

    @Override
    public void initialize() {
        /*shooter.setFlywheelRPM(limelight.calculateFlywheelRPM());
        shooter.setHoodHeading(limelight.calculateHoodHeading());*/
    }

    @Override
    public boolean isFinished() {
        return true;
        // return shooter.isFlywheelRevvedUp();
    }

}
