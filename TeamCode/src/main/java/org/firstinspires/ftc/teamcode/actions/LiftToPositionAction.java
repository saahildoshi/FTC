package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * ROAD RUNNER ACTION: MOVE LIFT TO AN ENCODER POSITION
 *
 * This Action is deliberately NON-BLOCKING. Road Runner calls run() repeatedly,
 * allowing this lift movement to participate correctly in ParallelAction.
 *
 * On the first loop it tells the lift motor where to go. On later loops it only
 * checks encoder progress. Returning true means "keep running"; returning false
 * means "this Action has finished."
 */
public final class LiftToPositionAction implements Action {
    private final LiftSubsystem lift;
    private final int targetTicks;
    private boolean started = false;

    public LiftToPositionAction(LiftSubsystem lift, int targetTicks) {
        this.lift = lift;
        this.targetTicks = targetTicks;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        // Command the motor only once when the Action begins.
        if (!started) {
            lift.moveTo(targetTicks);
            started = true;
        }

        // Dashboard values let students watch the physical lift approach target.
        packet.put("liftTargetTicks", lift.getTargetPosition());
        packet.put("liftCurrentTicks", lift.getCurrentPosition());

        // Keep the Action alive until the encoder reaches the allowed tolerance.
        return !lift.atTarget();
    }
}
