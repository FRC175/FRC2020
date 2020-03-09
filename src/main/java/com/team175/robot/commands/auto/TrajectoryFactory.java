package com.team175.robot.commands.auto;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;

import java.util.List;

public final class TrajectoryFactory {

    private static final DifferentialDriveVoltageConstraint voltageConstraint = new DifferentialDriveVoltageConstraint(
            new SimpleMotorFeedforward(Drive.KS, Drive.KV, Drive.KA),
            Drive.getInstance().getKinematics(),
            Drive.MAX_VOLTAGE
    );
    private static final TrajectoryConfig config = new TrajectoryConfig(Drive.MAX_VELOCITY, Drive.MAX_ACCELERATION)
            .setKinematics(Drive.getInstance().getKinematics())
            .addConstraint(voltageConstraint);
    private static final TrajectoryConfig reverseConfig = new TrajectoryConfig(Drive.MAX_VELOCITY, Drive.MAX_ACCELERATION)
            .setKinematics(Drive.getInstance().getKinematics())
            .addConstraint(voltageConstraint)
            .setReversed(true);

    public static Trajectory getAllianceTrenchRunNear() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(-3, -1.6, new Rotation2d(0)),
                        new Pose2d(-6.5, -1.6, new Rotation2d(0))
                ),
                // config
                reverseConfig
        );
    }

    public static Trajectory getAllianceTrenchRunMiddle() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(1, 1, new Rotation2d(90)),
                        new Pose2d(3, 3.3, new Rotation2d(0)),
                        new Pose2d(6.5, 3.3, new Rotation2d(0))
                ),
                config
        );
    }

    public static Trajectory getAllianceTrenchRunFar() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(1, 3.5, new Rotation2d(90)),
                        new Pose2d(3, 6.2, new Rotation2d(0)),
                        new Pose2d(6.5, 6.2, new Rotation2d(0))
                ),
                config
        );
    }

    public static Trajectory getAllianceTrenchReturnNear() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(-6.5, -1.6, new Rotation2d(0)),
                        new Pose2d(-3, -1.6, new Rotation2d(0))
                ),
                // reverseConfig
                config
        );
    }

    public static Trajectory getAllianceTrenchReturnMiddle() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(6.5, 3.3, new Rotation2d(0)),
                        new Pose2d(3, 3.3, new Rotation2d(0))
                ),
                reverseConfig
        );
    }

    public static Trajectory getAllianceTrenchReturnFar() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(6.5, 6.2, new Rotation2d(0)),
                        new Pose2d(3, 6.2, new Rotation2d(0))
                ),
                reverseConfig
        );
    }

    public static Trajectory getOpponentTrenchStealNear() {
        return TrajectoryGenerator.generateTrajectory(
                List.of(
                        new Pose2d(0, 0, new Rotation2d(0)),
                        new Pose2d(3.5, 0, new Rotation2d(0))
                ),
                // Pass config
                config
        );
    }

}
