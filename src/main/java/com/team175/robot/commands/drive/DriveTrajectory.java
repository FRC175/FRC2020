package com.team175.robot.commands.drive;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

public final class DriveTrajectory extends CommandBase {

    private final Drive drive;
    private final RamseteCommand ramsete;

    private static final double RAMSETE_B = 2;
    private static final double RAMSETE_ZETA = 0.7;
    private static final double VELOCITY_KP = 15;

    public DriveTrajectory(Trajectory trajectory, Drive drive) {
        this.drive = drive;
        // Run RAMSETE control loop on Talon SRX
        // NOTE: This is pretty finicky, but can be tuned with better VELOCITY_KP values. This is the ideal version
        //       though since the calculations are processed on a separate controller.
        /*this.ramsete = new RamseteCommand(
                trajectory,
                drive::getPose,
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                drive.getKinematics(),
                drive::setMetersPerSecond,
                drive
        );*/
        // RUN RAMSETE control loop on RoboRIO
        this.ramsete = new RamseteCommand(
                trajectory,
                drive::getPose,
                new RamseteController(RAMSETE_B, RAMSETE_ZETA),
                Drive.FEEDFORWARD,
                Drive.KINEMATICS,
                drive::getWheelSpeeds,
                new PIDController(VELOCITY_KP, 0, 0),
                new PIDController(VELOCITY_KP, 0, 0),
                drive::setVoltage,
                drive
        );
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
