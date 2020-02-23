package com.team175.robot.commands;

import com.team175.robot.subsystems.ColorWheelSpinner;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class SpinColorWheelToColor extends CommandBase {

    private final ColorWheelSpinner colorWheelSpinner;

    private String colorSetpoint;

    public SpinColorWheelToColor(ColorWheelSpinner colorWheelSpinner) {
        this.colorWheelSpinner = colorWheelSpinner;
    }

    private String getColorSetpoint(char fieldWantedColor) {
        switch (fieldWantedColor) {
            case 'B':
                return "Red";
            case 'G':
                return "Yellow";
            case 'R':
                return "Blue";
            case 'Y':
                return "Green";
            default:
                return "Unknown";
        }
    }

    @Override
    public void initialize() {
        String gameData = DriverStation.getInstance().getGameSpecificMessage();
        if (gameData.length() > 0) {
            colorSetpoint = getColorSetpoint(gameData.charAt(0));
        } else {
            colorSetpoint = "Unknown";
        }
    }

    @Override
    public void execute() {
        colorWheelSpinner.setOpenLoop(0.25);
    }

    @Override
    public void end(boolean interrupted) {
        colorWheelSpinner.setOpenLoop(0);
    }

    @Override
    public boolean isFinished() {
        return colorWheelSpinner.getStringColorMatch().equals(colorSetpoint);
    }

}
