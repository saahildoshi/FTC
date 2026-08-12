package org.firstinspires.ftc.teamcode.actions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * SIMPLE ROAD RUNNER ACTION EXAMPLE
 *
 * Road Runner Actions are small units of autonomous behavior. An Action returns
 * true while it still needs to run and false when it is finished.
 *
 * This action stops the drivetrain once and immediately finishes. It is simple
 * on purpose so students can see the Action pattern before building mechanism
 * actions such as LiftToPositionAction or OpenClawAction later.
 */
public final class StopDriveAction implements Action {
    private final DriveSubsystem drive;

    public StopDriveAction(DriveSubsystem drive) {
        this.drive = drive;
    }

    @Override
    public boolean run(@NonNull TelemetryPacket packet) {
        drive.stop();
        packet.put("action", "StopDriveAction");

        // false = this Action is complete; Road Runner may move to the next Action.
        return false;
    }
}
