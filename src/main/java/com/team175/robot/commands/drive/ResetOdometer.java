package com.team175.robot.commands.drive;

import com.team175.robot.subsystems.Drive;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

/**
 * ResetOdometer is a command used to reset the robot's odometer. This is usually done at the beginning of an autonomous
 * mode so that robot knows where it is relative to field.
 */
public final class ResetOdometer extends CommandBase {

    private final Pose2d pose;
    private final Drive drive;

    public ResetOdometer(Pose2d pose, Drive drive) {
        this.pose = pose;
        this.drive = drive;
    }

    @Override
    public void initialize() {
        drive.resetOdometer(pose);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
