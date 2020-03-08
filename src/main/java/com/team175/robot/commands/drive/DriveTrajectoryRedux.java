package com.team175.robot.commands.drive;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public class DriveTrajectoryRedux extends RamseteCommand {

    private static final double RAMSETE_B = 2;
    private static final double RAMSETE_ZETA = 0.7;
    private static final double VELOCITY_KP = 15;

    public DriveTrajectoryRedux(Trajectory trajectory, Drive drive) {
        super(
                trajectory,
                drive::getPose,
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                drive.getFeedforward(),
                drive.getKinematics(),
                drive::getWheelSpeeds,
                new PIDController(VELOCITY_KP, 0, 0),
                new PIDController(VELOCITY_KP, 0, 0),
                drive::setVoltage,
                drive
        );
    }

}
