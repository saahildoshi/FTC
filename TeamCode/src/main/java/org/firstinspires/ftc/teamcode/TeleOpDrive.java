package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Mecanum Drive (Road Runner 1.0)", group = "TeleOp")
public final class TeleOpDrive extends LinearOpMode {
    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0), Rotation2d.exp(0.0));

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, INITIAL_POSE);
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean fieldCentric = true;
        boolean previousY = false;
        boolean previousHeadingReset = false;
        long previousLoopTimeNs = System.nanoTime();
        double filteredLoopHz = 0.0;

        telemetry.addLine("Drivetrain initialized");
        telemetry.addData("Drive mode", "Field-Centric");
        telemetry.addLine("Y: toggle mode | OPTIONS/START: reset heading");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        previousLoopTimeNs = System.nanoTime();

        while (opModeIsActive()) {
            long loopTimeNs = System.nanoTime();
            double loopPeriodSeconds = (loopTimeNs - previousLoopTimeNs) * 1e-9;
            previousLoopTimeNs = loopTimeNs;
            if (loopPeriodSeconds > 0.0) {
                double instantaneousLoopHz = 1.0 / loopPeriodSeconds;
                filteredLoopHz = filteredLoopHz == 0.0
                        ? instantaneousLoopHz
                        : 0.90 * filteredLoopHz + 0.10 * instantaneousLoopHz;
            }

            boolean yPressed = gamepad1.y;
            if (yPressed && !previousY) {
                fieldCentric = !fieldCentric;
            }
            previousY = yPressed;

            boolean headingResetPressed = gamepad1.options || gamepad1.start;
            if (headingResetPressed && !previousHeadingReset) {
                drive.resetHeading();
            }
            previousHeadingReset = headingResetPressed;

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            Vector2d translation = new Vector2d(
                    gamepad1.left_stick_y,
                    gamepad1.left_stick_x);
            if (fieldCentric) {
                translation = pose.heading.inverse().times(translation);
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    translation,
                    -gamepad1.right_stick_x));

            telemetry.addData("Drive mode",
                    fieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Pose X (in)", "%.2f", pose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", pose.position.y);
            telemetry.addData("Heading (deg)", "%.1f",
                    Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData("Loop rate (Hz)", "%.1f", filteredLoopHz);
            telemetry.update();
        }

        drive.setDrivePowers(new PoseVelocity2d(
                new Vector2d(0.0, 0.0), 0.0));
    }
}
