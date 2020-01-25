package com.team175.robot.models;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * XboxButton is an extension to {@link edu.wpi.first.wpilibj2.command.button.Button} that uses Xbox controller button
 * names instead of button numbers.
 */
public class XboxButton extends Button {

    private final AldrinXboxController controller;

    private AldrinXboxController.Button button;
    private AldrinXboxController.DPad dPadButton;

    public XboxButton(AldrinXboxController controller, AldrinXboxController.Button button) {
        this.controller = controller;
        this.button = button;
    }

    public XboxButton(AldrinXboxController controller, AldrinXboxController.DPad dPadButton) {
        this.controller = controller;
        this.dPadButton = dPadButton;
    }

    @Override
    public boolean get() {
        return dPadButton == null ? controller.getRawButton(button.value) : controller.getPOV() == dPadButton.value;
    }

}