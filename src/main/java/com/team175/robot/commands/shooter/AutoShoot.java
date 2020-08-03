package com.team175.robot.commands.shooter;

import com.team175.robot.subsystems.Intake;
import com.team175.robot.subsystems.Limelight;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class AutoShoot extends SequentialCommandGroup {

    private final Shooter shooter;
    private final Intake intake;

    public AutoShoot(Shooter shooter, Limelight limelight, Intake intake) {
        this.shooter = shooter;
        this.intake = intake;

        addCommands(
                // new RotateTurretToTarget(shooter, limelight),
                // new RevUpShooter(shooter, limelight),
                // new ShootBalls(shooter, intake)
                new InstantCommand(() -> {
                    shooter.setBallGate(true);
                    shooter.setFlywheelOpenLoop(0.5);
                    shooter.setHoodHeading(Rotation2d.fromDegrees(90));
                }, shooter),
                new WaitCommand(5),
                new InstantCommand(() -> {
                    shooter.setBallGate(false);
                    shooter.setFlywheelOpenLoop(0);
                    shooter.setHoodHeading(Rotation2d.fromDegrees(0));
                }, shooter)
        );
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        shooter.setHoodHeading(Rotation2d.fromDegrees(0));
        shooter.setFlywheelOpenLoop(0);
        intake.setIndexerOpenLoop(0);
    }

}
