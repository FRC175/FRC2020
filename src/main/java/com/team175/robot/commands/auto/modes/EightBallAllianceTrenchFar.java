package com.team175.robot.commands.auto.modes;

import com.team175.robot.commands.auto.TrajectoryFactory;
import com.team175.robot.commands.drive.DriveTrajectory;
import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class EightBallAllianceTrenchFar extends SequentialCommandGroup {

    public EightBallAllianceTrenchFar(Drive drive) {
        addCommands(
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchRunFar(), drive),
                new WaitCommand(0.5),
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchReturnFar(), drive),
                new InstantCommand(() -> drive.setOpenLoop(0, 0))
        );
    }

}
