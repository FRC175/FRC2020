package com.team175.robot.commands.auto.modes;

import com.team175.robot.commands.auto.TrajectoryFactory;
import com.team175.robot.commands.drive.DriveTrajectory;
import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class EightBallAllianceTrenchMiddle extends SequentialCommandGroup {

    public EightBallAllianceTrenchMiddle(Drive drive) {
        addCommands(
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchRunMiddle(), drive),
                new WaitCommand(0.5),
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchReturnMiddle(), drive),
                new InstantCommand(() -> drive.setOpenLoop(0, 0))
        );
    }

}
