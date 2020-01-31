package com.team175.robot.models;

import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * XboxButton is an extension to {@link edu.wpi.first.wpilibj2.command.button.Button} that uses Xbox controller button
 * names instead of button numbers.
 */
public class XboxButton extends Button {

    private final AdvancedXboxController controller;

    private AdvancedXboxController.Button button;
    private AdvancedXboxController.DPad dPadButton;
    private AdvancedXboxController.Trigger trigger;

    public XboxButton(AdvancedXboxController controller, AdvancedXboxController.Button button) {
        this.controller = controller;
        this.button = button;
    }

    public XboxButton(AdvancedXboxController controller, AdvancedXboxController.DPad dPadButton) {
        this.controller = controller;
        this.dPadButton = dPadButton;
    }

    public XboxButton(AdvancedXboxController controller, AdvancedXboxController.Trigger trigger) {
        this.controller = controller;
        this.trigger = trigger;
    }

    @Override
    public boolean get() {
        if (button != null) {
            return controller.getRawButton(button.value);
        } else if (dPadButton != null) {
            return controller.getPOV() == dPadButton.value;
        } else {
            return controller.getRawAxis(trigger.value) > 0.1;
        }
    }

}