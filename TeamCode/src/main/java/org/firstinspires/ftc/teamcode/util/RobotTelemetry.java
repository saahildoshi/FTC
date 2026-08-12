package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Pose2d;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;

/**
 * TELEMETRY HELPER
 *
 * Telemetry is the robot's dashboard for the drive team and programmers. It
 * sends values to the Driver Station so students can compare what the software
 * believes is happening with what the physical robot is actually doing.
 */
public final class RobotTelemetry {
    private final Telemetry telemetry;

    public RobotTelemetry(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    /** Show Road Runner's current field-position estimate. */
    public void addDriveData(DriveSubsystem drive) {
        Pose2d pose = drive.updateAndGetPose();
        telemetry.addData("Pose X", pose.position.x);
        telemetry.addData("Pose Y", pose.position.y);
        telemetry.addData("Heading (deg)", Math.toDegrees(pose.heading.toDouble()));
    }

    /** Push all queued values to the Driver Station. */
    public void update() {
        telemetry.update();
    }
}
