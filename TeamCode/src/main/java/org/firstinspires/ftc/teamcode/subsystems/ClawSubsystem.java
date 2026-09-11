package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * ACTIVE CLAW / ARM SERVO SUBSYSTEM
 *
 * Competition claw positions come from RobotConstants so TeleOp and autonomous
 * always use the same fixed home/closed and open positions.
 */
public final class ClawSubsystem {
    private final Servo claw;

    public ClawSubsystem(RobotHardware robot) {
        claw = robot.claw;
    }

    public void open() {
        claw.setPosition(RobotConstants.Claw.OPEN_POSITION);
    }

    public void close() {
        claw.setPosition(RobotConstants.Claw.CLOSED_POSITION);
    }

    public double getPosition() {
        return claw.getPosition();
    }
}
