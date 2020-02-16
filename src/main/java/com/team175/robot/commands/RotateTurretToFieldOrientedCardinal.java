package com.team175.robot.commands;

import com.team175.robot.positions.TurretCardinal;
import com.team175.robot.subsystems.Drive;
import com.team175.robot.subsystems.Shooter;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class RotateTurretToFieldOrientedCardinal extends CommandBase {

    private final Drive drive;
    private final Shooter shooter;
    private final TurretCardinal cardinal;

    public RotateTurretToFieldOrientedCardinal(Drive drive, Shooter shooter, TurretCardinal cardinal) {
        this.drive = drive;
        this.shooter = shooter;
        this.cardinal = cardinal;
        addRequirements(drive, shooter);
    }

    @Override
    public void initialize() {
        // Thank you team 254!
        // theta => gyro
        // phi => turret
        // lambda => cardinal
        // (-(theta + phi) + lambda) + phi

        // IDK if I really need to use odometer to get rotation when I can just use gyro reading
        /*Rotation2d heading = drive.getPose().getRotation()
                .rotateBy(shooter.getTurretHeading())
                .unaryMinus()
                .rotateBy(cardinal.toRotation2d())
                .rotateBy(shooter.getTurretHeading());*/

        // Rotation2d heading = drive.getHeading()
        //         .rotateBy(shooter.getTurretHeading())
        //         .unaryMinus()
        //         .rotateBy(cardinal.toRotation2d())
        //         .rotateBy(shooter.getTurretHeading());

        // TODO: Adjust heading to accommodate for lack of full 360 rotation (i.e. software limits)
        /*if (mGoal.state.turret < Constants.kTurretConstants.kMinUnitsLimit) {
            mGoal.state.turret += 360.0;
        }
        if (mGoal.state.turret > Constants.kTurretConstants.kMaxUnitsLimit) {
            mGoal.state.turret -= 360.0;
        }*/
        // shooter.setTurretHeading(heading);
    }

    @Override
    public boolean isFinished() {
        return true;
    }

}
