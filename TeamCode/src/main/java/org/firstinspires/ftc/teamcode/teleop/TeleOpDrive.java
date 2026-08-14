package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;


@TeleOp(name = "Mecanum Drive (Road Runner 1.0)", group = "TeleOp")
public final class TeleOpDrive extends LinearOpMode {

    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0));



    @Override
    public void runOpMode() throws InterruptedException {



        // ---------------- Controls ----------------
        boolean slowMode = false;
        boolean previousA = false;

        boolean fieldCentric = false;
        boolean previousY = false;
        boolean previousHeadingReset = false;

        // ---------------- Drive ----------------

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, INITIAL_POSE);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        // ---------------- Heading Hold PID ----------------

        boolean headingHold = false;
        double targetHeading = 0.0;

        double headingKp = 1.5;
        double headingKi = 0.0;
        double headingKd = 0.05;

        double headingIntegral = 0.0;
        double previousHeadingError = 0.0;
        long previousHeadingTimeNs = System.nanoTime();

        long lastManualRotationTime = System.nanoTime();

        final double HEADING_HOLD_DELAY = 0.2;

        // ---------------- Acceleration Limiting ----------------

        double currentX = 0.0;
        double currentY = 0.0;
        double currentRotation = 0.0;

        final double DRIVE_ACCEL = 3.5;
        final double ROTATION_ACCEL = 3.0;

        // ---------------- Loop Timing ----------------

        long previousLoopTimeNs;
        double filteredLoopHz = 0.0;

        // ---------------- Telemetry ----------------

        telemetry.addLine("Drivetrain initialized");
        telemetry.addData("Drive mode", "Robot-Centric");
        telemetry.addLine("Y: toggle mode | OPTIONS/START: reset heading");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        previousLoopTimeNs = System.nanoTime();

        while (opModeIsActive()) {

            // =========================================================
            // LOOP TIMING
            // =========================================================

            long loopTimeNs = System.nanoTime();

            double loopPeriodSeconds =
                    (loopTimeNs - previousLoopTimeNs) * 1e-9;

            previousLoopTimeNs = loopTimeNs;

            if (loopPeriodSeconds > 0.0) {

                double instantaneousLoopHz =
                        1.0 / loopPeriodSeconds;

                filteredLoopHz =
                        filteredLoopHz == 0.0
                                ? instantaneousLoopHz
                                : 0.90 * filteredLoopHz
                                  + 0.10 * instantaneousLoopHz;
            }

            // =========================================================
            // FIELD / ROBOT CENTRIC TOGGLE
            // =========================================================

            boolean yPressed = gamepad1.y;

            if (yPressed && !previousY) {
                fieldCentric = !fieldCentric;
            }

            previousY = yPressed;

            // =========================================================
            // HEADING RESET
            // =========================================================

            boolean headingResetPressed =
                    gamepad1.options || gamepad1.start;

            if (headingResetPressed && !previousHeadingReset) {

                drive.resetHeading();

                headingHold = false;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            previousHeadingReset = headingResetPressed;

            // =========================================================
            // SLOW MODE
            // =========================================================

            boolean aPressed = gamepad1.a;

            if (aPressed && !previousA) {
                slowMode = !slowMode;
            }

            previousA = aPressed;

            // =========================================================
            // RETURN HOME
            // X = (0, 0, 0)
            // =========================================================

            if (gamepad1.x) {

                telemetry.addLine("Returning home...");
                telemetry.update();

                Pose2d currentPose =
                        drive.updateAndGetPose();

                Action returnHome =
                        drive.roadRunner()
                                .actionBuilder(currentPose)
                                .strafeToLinearHeading(new Vector2d(0, 0), Math.toRadians(0))
                                .build();

                Actions.runBlocking(returnHome);

                // After returning home, heading target is 0 degrees
                targetHeading = 0.0;

                headingHold = true;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;

                // Reset acceleration state
                currentX = 0.0;
                currentY = 0.0;
                currentRotation = 0.0;
            }

            // =========================================================
            // UPDATE LOCALIZATION
            // =========================================================
            Pose2d pose =
                    drive.updateAndGetPose();

            // =========================================================
            // RAW JOYSTICK INPUT
            // =========================================================

            double targetX = -gamepad1.left_stick_y;
            double targetY = -gamepad1.left_stick_x;
            double targetRotation = -gamepad1.right_stick_x;

            // =========================================================
            // ACCELERATION LIMITING
            // =========================================================

            double maxDriveChange =
                    DRIVE_ACCEL * loopPeriodSeconds;

            double maxRotationChange =
                    ROTATION_ACCEL * loopPeriodSeconds;

            currentX += Math.max(
                    -maxDriveChange,
                    Math.min(
                            maxDriveChange,
                            targetX - currentX));

            currentY += Math.max(
                    -maxDriveChange,
                    Math.min(
                            maxDriveChange,
                            targetY - currentY));

            currentRotation += Math.max(
                    -maxRotationChange,
                    Math.min(
                            maxRotationChange,
                            targetRotation - currentRotation));

            // =========================================================
            // ROTATION / HEADING HOLD
            // =========================================================

            boolean manuallyRotating =
                    Math.abs(targetRotation) > 0.05;

            if (manuallyRotating) {

                lastManualRotationTime =
                        System.nanoTime();

                headingHold = false;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            double timeSinceManualRotation =
                    (System.nanoTime()
                            - lastManualRotationTime) * 1e-9;

            /*
             * Wait until:
             *
             * 1. Driver has released rotation
             * 2. The delay has passed
             * 3. The rotation ramp has nearly stopped
             *
             * This prevents the PID from fighting the
             * acceleration limiter.
             */

            boolean rotationStopped =
                    Math.abs(currentRotation) < 0.02;

            if (!manuallyRotating
                    && rotationStopped
                    && !headingHold
                    && timeSinceManualRotation >= HEADING_HOLD_DELAY) {

                headingHold = true;

                targetHeading =
                        pose.heading.toDouble();

                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            // =========================================================
            // HEADING PID
            // =========================================================

            double headingCorrection = 0.0;

            if (headingHold) {

                double currentHeading =
                        pose.heading.toDouble();

                double headingError =
                        targetHeading - currentHeading;

                // Wrap heading error to -PI ... +PI
                while (headingError > Math.PI) {
                    headingError -= 2.0 * Math.PI;
                }

                while (headingError < -Math.PI) {
                    headingError += 2.0 * Math.PI;
                }

                long currentTimeNs =
                        System.nanoTime();

                double dt =
                        (currentTimeNs
                                - previousHeadingTimeNs) * 1e-9;

                previousHeadingTimeNs =
                        currentTimeNs;

                if (dt > 0.0 && dt < 0.1) {

                    headingIntegral +=
                            headingError * dt;

                    double derivative =
                            (headingError
                                    - previousHeadingError)
                                    / dt;

                    headingCorrection =
                            headingKp * headingError
                                    + headingKi * headingIntegral
                                    + headingKd * derivative;

                    // Limit PID correction
                    headingCorrection =
                            Math.max(
                                    -0.5,
                                    Math.min(
                                            0.5,
                                            headingCorrection));
                }

                previousHeadingError =
                        headingError;
            }
            // =========================================================
            // SPEED MULTIPLIER
            // =========================================================

            double speedMultiplier =
                    slowMode ? 0.4 : 1.0;
            // =========================================================
            // ROTATION POWER
            // =========================================================

            double rotationPower;

            if (manuallyRotating || !headingHold) {

                // Driver controls rotation
                rotationPower =
                        currentRotation * speedMultiplier;

            } else {

                // Heading PID controls rotation
                rotationPower =
                        headingCorrection;
            }

            // =========================================================
// DRIVE
// =========================================================

            double outputX = currentX * speedMultiplier;
            double outputY = currentY * speedMultiplier;
            double outputRotation = rotationPower;

            if (fieldCentric) {
                drive.fieldCentric(outputX, outputY, outputRotation);
            } else {
                drive.robotCentric(outputX, outputY, outputRotation);
            }

            // =========================================================
            // TELEMETRY
            // =========================================================

            telemetry.addData(
                    "Drive mode",
                    fieldCentric
                            ? "Field-Centric"
                            : "Robot-Centric");

            telemetry.addData(
                    "Slow Mode",
                    slowMode);

            telemetry.addData(
                    "Pose X (in)",
                    "%.2f",
                    pose.position.x);

            telemetry.addData(
                    "Pose Y (in)",
                    "%.2f",
                    pose.position.y);

            telemetry.addData(
                    "Heading (deg)",
                    "%.1f",
                    Math.toDegrees(
                            pose.heading.toDouble()));

            telemetry.addData(
                    "Target Heading (deg)",
                    "%.1f",
                    Math.toDegrees(targetHeading));

            telemetry.addData(
                    "Heading Hold",
                    headingHold);

            telemetry.addData(
                    "Drive X",
                    "%.2f",
                    currentX);

            telemetry.addData(
                    "Drive Y",
                    "%.2f",
                    currentY);

            telemetry.addData(
                    "Rotation",
                    "%.2f",
                    currentRotation);

            telemetry.addData(
                    "Loop rate (Hz)",
                    "%.1f",
                    filteredLoopHz);

            telemetry.update();
        }
        drive.stop();
    }
}