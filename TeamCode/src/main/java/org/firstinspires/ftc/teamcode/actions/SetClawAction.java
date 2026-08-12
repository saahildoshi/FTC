package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;

/**
 * ROAD RUNNER ACTION: OPEN OR CLOSE THE CLAW
 *
 * Servo commands are effectively immediate from the program's point of view:
 * Java tells the servo controller its new target and the servo then physically
 * travels there. This Action therefore sends the command once and finishes.
 */
public final class SetClawAction implements Action {
    private final ClawSubsystem claw;
    private final boolean open;

    /** @param open true opens the claw; false closes it. */
    public SetClawAction(ClawSubsystem claw, boolean open) {
        this.claw = claw;
        this.open = open;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        if (open) {
            claw.open();
            packet.put("clawCommand", "OPEN");
        } else {
            claw.close();
            packet.put("clawCommand", "CLOSED");
        }

        // false tells Road Runner this one-step Action is complete.
        return false;
    }
}
