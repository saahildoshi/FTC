package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.LiftConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * ACTIVE CLAW / ARM SERVO SUBSYSTEM
 *
 * Servo positions come from LiftConstants so they can be tuned live in FTC Dashboard.
 */
public final class ClawSubsystem {
    private final Servo claw;

    public ClawSubsystem(RobotHardware robot) {
        claw = robot.claw;
    }

    public void open() {
        claw.setPosition(LiftConstants.CLAW_OPEN_POSITION);
    }

    public void close() {
        claw.setPosition(LiftConstants.CLAW_CLOSED_POSITION);
    }

    public double getPosition() {
        return claw.getPosition();
    }
}
