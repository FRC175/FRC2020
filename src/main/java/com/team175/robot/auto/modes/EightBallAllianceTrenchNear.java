package com.team175.robot.auto.modes;

import com.team175.robot.auto.TrajectoryFactory;
import com.team175.robot.commands.drive.DriveTrajectory;
import com.team175.robot.subsystems.Drive;
import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class EightBallAllianceTrenchNear extends SequentialCommandGroup {

    public EightBallAllianceTrenchNear(Drive drive, Shooter shooter, Limelight limelight, Intake intake) {
        addCommands(
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchRunNear(), drive),
                new WaitCommand(0.5),
                new DriveTrajectory(TrajectoryFactory.getAllianceTrenchReturnNear(), drive),
                new InstantCommand(() -> drive.setOpenLoop(0, 0))
                /*new LogCommand("Beginning eight ball auto"),
                new AutoShoot(shooter, limelight, intake),
                parallel(
                        // new LogCommand("Bring down intake arm and turn it on"),
                        new InstantCommand(() -> {
                            intake.deploy();
                            intake.setIndexerOpenLoop(1);
                            intake.setRollerOpenLoop(1);
                        }, intake),
                        new WaitCommand(5)
                        // new LogCommand("Drive path to pick up 5 balls in trench")
                ),
                // new LogCommand("Drive path to shooting position"),
                new AutoShoot(shooter, limelight, intake),
                new InstantCommand(() -> {
                    intake.retract();
                    intake.setIndexerOpenLoop(0);
                    intake.setRollerOpenLoop(0);
                }, intake)*/
        );
    }

}
