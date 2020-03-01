package com.team175.robot.commands.auto;

import com.team175.robot.commands.shooter.AutoShoot;
import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class ThreeBallAutoClose extends SequentialCommandGroup {

    public ThreeBallAutoClose(Shooter shooter, Limelight limelight, Intake intake) {
        addCommands(new AutoShoot(shooter, limelight, intake));
    }

}
