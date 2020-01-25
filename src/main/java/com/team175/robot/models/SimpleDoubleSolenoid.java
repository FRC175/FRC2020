package com.team175.robot.models;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

/**
 * SimpleDoubleSolenoid treats a double-acting solenoid as a single-acting one.
 */
public class SimpleDoubleSolenoid {

    /**
     * The double-acting solenoid
     */
    private final DoubleSolenoid solenoid;

    public SimpleDoubleSolenoid(int forwardChannel, int reverseChannel, int pcmID) {
        solenoid = new DoubleSolenoid(pcmID, forwardChannel, reverseChannel);
    }

    public SimpleDoubleSolenoid(int forwardChannel, int reverseChannel) {
        solenoid = new DoubleSolenoid(forwardChannel, reverseChannel);
    }

    public void set(boolean enable) {
        solenoid.set(enable ? Value.kForward : Value.kReverse);
    }

    public boolean get() {
        return solenoid.get() == Value.kForward;
    }

}