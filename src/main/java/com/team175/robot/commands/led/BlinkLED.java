package com.team175.robot.commands.led;

import com.team175.robot.subsystems.LED;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;

public final class BlinkLED extends CommandBase {

    private final LED led;
    private final Color blinkColor;

    private Color previousColor;
    private double startTime;
    private double time;

    private static final double BLINK_CYCLE_DURATION = 0.2; // seconds for each blink cycle
    private static final double BLINK_TIME = 1; // Total time in blink

    public BlinkLED(LED led, Color color) {
        this.led = led;
        blinkColor = color;
        addRequirements(led);
    }

    @Override
    public void initialize() {
        previousColor = led.getColor();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        time = Timer.getFPGATimestamp() - startTime;

        // Every 0.1 s, turn on the LED
        // A blink cycle is 0.2 s
        int cycleNum = (int) (time / (BLINK_CYCLE_DURATION / 2.0));
        if ((cycleNum % 2) == 0) {
            led.setColor(blinkColor);
        } else {
            led.setColor(Color.kBlack); // Off
        }
    }

    @Override
    public void end(boolean interrupted) {
        led.setColor(previousColor);
    }

    @Override
    public boolean isFinished() {
        return time > BLINK_TIME;
    }

}
