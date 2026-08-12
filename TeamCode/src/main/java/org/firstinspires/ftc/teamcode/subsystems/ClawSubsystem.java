package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * ACTIVE CLAW SUBSYSTEM
 *
 * This class owns the physical positional servo named "claw" in RobotHardware.
 * A positional servo moves to a commanded position between 0.0 and 1.0.
 *
 * The actual linkage determines which numerical positions are safe. The current
 * open/closed values are starting values and must be checked on the real robot.
 */
public final class ClawSubsystem {
    private final Servo clawServo;

    /** Reuse the servo that RobotHardware already mapped from the Control Hub. */
    public ClawSubsystem(RobotHardware robot) {
        clawServo = robot.claw;
    }

    /** Move the servo to the configured open position to release an object. */
    public void open() {
        clawServo.setPosition(RobotConstants.Claw.OPEN_POSITION);
    }

    /** Move the servo to the configured closed position to hold an object. */
    public void close() {
        clawServo.setPosition(RobotConstants.Claw.CLOSED_POSITION);
    }

    /** Return the servo's last commanded position for telemetry/debugging. */
    public double getPosition() {
        return clawServo.getPosition();
    }
}
