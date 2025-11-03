package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.impl.ServoEx;

public class Light implements Subsystem {
    public static final Light INSTANCE = new Light();

    // Servo positions for the goBILDA RGB Light (approximate common values)
    // These specific values correspond to persistent, solid colors.
    public static final double OFF_POSITION = 0.5;
    public static final double SOLID_RED_POSITION = 0.7;
    public static final double SOLID_GREEN_POSITION = 0.6;
    public static final double SOLID_BLUE_POSITION = 0.8;

    // Renamed from 'brightness' to 'targetPosition' to reflect the hardware
    private double targetPosition = OFF_POSITION; // Default to OFF or safe state

    private final ServoEx servo = new ServoEx("servo1");
    private Light() { }

    @Override
    public void periodic() {
        // The periodic loop constantly applies the target position, ensuring the color/pattern stays active
        servo.setPosition(targetPosition);
    }

    @Override
    public void initialize() {
        // We will command the light to turn on in the TeleOp code instead of here
        // The periodic() loop will handle applying the default 'targetPosition'
    }

    // New method name: setPattern, which takes the specific servo position constant
    public Command setPattern(double position) {
        return new InstantCommand(() -> {
            this.targetPosition = position;
        }).requires(this);
    }
}
