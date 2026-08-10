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

        boolean slowMode = false;
        boolean previousA = false;

        MecanumDrive drive = new MecanumDrive(hardwareMap, INITIAL_POSE);
        telemetry = new MultipleTelemetry(
                telemetry, FtcDashboard.getInstance().getTelemetry());

        boolean fieldCentric = false;
        boolean previousY = false;
        boolean previousHeadingReset = false;

        boolean headingHold = false;
        double targetHeading = 0.0;


        double headingKp = 1.5;
        double headingKi = 0.0;
        double headingKd = 0.05;

        double headingIntegral = 0.0;
        double previousHeadingError = 0.0;
        long previousHeadingTimeNs = System.nanoTime();

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

                headingHold = false;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }
            previousHeadingReset = headingResetPressed;

            boolean aPressed = gamepad1.a;

            if (aPressed && !previousA) {
                slowMode = !slowMode;
            }

            previousA = aPressed;

            drive.updatePoseEstimate();
            Pose2d pose = drive.localizer.getPose();

            Vector2d translation = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x);

            double rotationInput = -gamepad1.right_stick_x;

            boolean rotationCommanded = Math.abs(rotationInput) > 0.05;

            if (rotationCommanded) {
                headingHold = false;
                headingIntegral = 0.0;
            } else if (!headingHold) {
                headingHold = true;
                targetHeading = pose.heading.toDouble();
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            double headingCorrection = 0.0;

            if (headingHold) {
                double currentHeading = pose.heading.toDouble();
                double headingError = targetHeading - currentHeading;

                while (headingError > Math.PI) {
                    headingError -= 2.0 * Math.PI;
                }

                while (headingError < -Math.PI) {
                    headingError += 2.0 * Math.PI;
                }

                long currentTimeNs = System.nanoTime();
                double dt = (currentTimeNs - previousHeadingTimeNs) * 1e-9;
                previousHeadingTimeNs = currentTimeNs;

                if (dt > 0.0 && dt < 0.1) {
                    headingIntegral += headingError * dt;

                    double derivative = (headingError - previousHeadingError) / dt;

                    headingCorrection =
                            headingKp * headingError
                                    + headingKi * headingIntegral
                                    + headingKd * derivative;

                    headingCorrection = Math.max(-0.5, Math.min(0.5, headingCorrection));
                }

                previousHeadingError = headingError;

            }



            if (fieldCentric) {
                translation = pose.heading.inverse().times(translation);
            }



            double speedMultiplier = slowMode ? 0.4 : 1.0;

            translation = translation.times(speedMultiplier);

            double rotationPower;

            if (rotationCommanded) {
                rotationPower = rotationInput * speedMultiplier;
            } else {
                rotationPower = headingCorrection;
            }

            drive.setDrivePowers(new PoseVelocity2d(
                    translation,
                    rotationPower));

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
