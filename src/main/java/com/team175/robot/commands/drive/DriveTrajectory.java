package com.team175.robot.commands.drive;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class DriveTrajectory extends CommandBase {

    private final Drive drive;
    private final SimpleMotorFeedforward feedforward;
    private final RamseteCommand ramsete;

    private static final double RAMSETE_B = 2;
    private static final double RAMSETE_ZETA = 0.7;
    private static final double VELOCITY_KP = 15;

    public DriveTrajectory(Trajectory trajectory, Drive drive) {
        this.drive = drive;
        this.feedforward = new SimpleMotorFeedforward(Drive.KS, Drive.KV, Drive.KA);
        this.ramsete = new RamseteCommand(
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
        /*this.ramsete = new RamseteCommand(
                trajectory,
                drive::getPose,
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                drive.getKinematics(),
                drive::setMetersPerSecond,
                drive
        );*/
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        ramsete.initialize();
    }

    @Override
    public void execute() {
        ramsete.execute();
    }

    @Override
    public void end(boolean interrupted) {
        ramsete.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return ramsete.isFinished();
    }

}
