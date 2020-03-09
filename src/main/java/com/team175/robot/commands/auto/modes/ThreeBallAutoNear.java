package com.team175.robot.commands.auto.modes;

import com.team175.robot.commands.shooter.AutoShoot;
import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class ThreeBallAutoNear extends SequentialCommandGroup {

    public ThreeBallAutoNear(Shooter shooter, Limelight limelight, Intake intake) {
        addCommands(new AutoShoot(shooter, limelight, intake));
    }

}
