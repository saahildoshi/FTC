package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * COMPETITION TELEOP
 *
 * GAMEPAD 1 = DRIVETRAIN ONLY
 * - Left stick: forward/back + strafe
 * - Right stick X: rotate
 * - A: toggle slow mode
 * - Y: toggle robot-centric / field-centric
 * - X: Road Runner return to (0, 0, 0)
 * - OPTIONS/START: reset heading
 *
 * GAMEPAD 2 = SUBSYSTEMS ONLY
 * - Right trigger: intake forward
 * - Left trigger: intake reverse
 * - D-pad Up: manually raise lift
 * - D-pad Down: manually lower lift
 * - Y: automatically move lift to HIGH, then open claw
 * - Left bumper: close claw
 * - Right bumper: open claw
 * - A: move 270 servo to 0.60 for 1 second, then return to 0.15
 *
 * The automatic lift sequence and 270 servo sequence are non-blocking so the
 * drivetrain and other subsystem controls continue updating while they run.
 */
@Config
@TeleOp(name = "Mecanum Drive (Road Runner 1.0)", group = "TeleOp")
public final class TeleOpDrive extends LinearOpMode {

    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0));

    // Dashboard-adjustable intake power, matching IntakeMotorTest behavior.
    public static double INTAKE_TEST_POWER = 0.50;

    // Calibrated FTC Servo commands. Servo.setPosition() uses a normalized
    // [0, 1] command rather than a physical angle in degrees.
    public static double SERVO_270_ZERO_POSITION = 0.15;
    public static double SERVO_270_OPEN_POSITION = 0.60;
    public static double SERVO_270_HOLD_SECONDS = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // =========================================================
        // HARDWARE / SUBSYSTEMS
        // =========================================================

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, INITIAL_POSE);

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(robot);
        LiftSubsystem lift = new LiftSubsystem(robot);
        ClawSubsystem claw = new ClawSubsystem(robot);
        Servo servo270 = robot.servo270;

        servo270.setDirection(Servo.Direction.REVERSE);
        servo270.setPosition(SERVO_270_ZERO_POSITION);
        claw.close();

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        // =========================================================
        // GAMEPAD 1 - DRIVE CONTROLS
        // =========================================================

        boolean slowMode = false;
        boolean previousA = false;

        boolean fieldCentric = false;
        boolean previousY = false;
        boolean previousHeadingReset = false;
        boolean previousX = false;

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
        final double HEADING_HOLD_DELAY = 0.8;

        // ---------------- Acceleration Limiting ----------------

        double currentX = 0.0;
        double currentY = 0.0;
        double currentRotation = 0.0;

        final double DRIVE_ACCEL = 3.5;
        final double ROTATION_ACCEL = 3.0;

        // =========================================================
        // GAMEPAD 2 - SUBSYSTEM STATE
        // =========================================================

        boolean previousServoA = false;
        boolean servo270CycleActive = false;
        long servo270OpenedTimeNs = 0L;

        boolean previousHighY = false;
        boolean autoHighActive = false;
        boolean autoHighClawOpened = false;

        // ---------------- Loop Timing ----------------

        long previousLoopTimeNs;
        double filteredLoopHz = 0.0;

        telemetry.addLine("TeleOp initialized");
        telemetry.addLine("GAMEPAD 1: drive only");
        telemetry.addLine("GAMEPAD 2: mechanisms only");
        telemetry.addLine("GP2 RT/LT: intake forward/reverse");
        telemetry.addLine("GP2 D-pad Up/Down: lift raise/lower");
        telemetry.addLine("GP2 Y: lift HIGH -> open claw");
        telemetry.addLine("GP2 LB/RB: claw close/open");
        telemetry.addLine("GP2 A: 270 servo 0.60 -> 1 sec -> 0.15");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            intake.stop();
            lift.stop();
            drive.stop();
            servo270.setPosition(SERVO_270_ZERO_POSITION);
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
                double instantaneousLoopHz = 1.0 / loopPeriodSeconds;

                filteredLoopHz =
                        filteredLoopHz == 0.0
                                ? instantaneousLoopHz
                                : 0.90 * filteredLoopHz
                                + 0.10 * instantaneousLoopHz;
            }

            // =========================================================
            // GAMEPAD 1 - FIELD / ROBOT CENTRIC TOGGLE
            // =========================================================

            boolean yPressed = gamepad1.y;

            if (yPressed && !previousY) {
                fieldCentric = !fieldCentric;
            }

            previousY = yPressed;

            // =========================================================
            // GAMEPAD 1 - HEADING RESET
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
            // GAMEPAD 1 - SLOW MODE
            // =========================================================

            boolean aPressed = gamepad1.a;

            if (aPressed && !previousA) {
                slowMode = !slowMode;
            }

            previousA = aPressed;

            // =========================================================
            // GAMEPAD 1 - RETURN HOME
            // X = (0, 0, 0)
            // =========================================================

            boolean xPressed = gamepad1.x;

            if (xPressed && !previousX) {
                telemetry.addLine("Returning drivetrain home...");
                telemetry.update();

                Pose2d currentPose = drive.updateAndGetPose();

                Action returnHome =
                        drive.roadRunner()
                                .actionBuilder(currentPose)
                                .strafeToLinearHeading(
                                        new Vector2d(0, 0),
                                        Math.toRadians(0))
                                .build();

                Actions.runBlocking(returnHome);

                targetHeading = 0.0;
                headingHold = true;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;

                currentX = 0.0;
                currentY = 0.0;
                currentRotation = 0.0;
            }

            previousX = xPressed;

            // =========================================================
            // GAMEPAD 2 - INTAKE TEST CONTROL
            // =========================================================

            double intakePower = 0.0;

            if (gamepad2.right_trigger > 0.10) {
                intakePower = Math.abs(INTAKE_TEST_POWER);
            } else if (gamepad2.left_trigger > 0.10) {
                intakePower = -Math.abs(INTAKE_TEST_POWER);
            }

            intake.setPower(intakePower);

            // =========================================================
            // GAMEPAD 2 - AUTOMATIC HIGH LIFT SEQUENCE
            // Y = close claw -> move to HIGH -> open claw at target.
            // D-pad lift input cancels automatic control immediately.
            // =========================================================

            boolean highYNow = gamepad2.y;
            boolean highYPressed = highYNow && !previousHighY;

            boolean raiseLift = gamepad2.dpad_up;
            boolean lowerLift = gamepad2.dpad_down;

            if (highYPressed && lift.isHomed()) {
                claw.close();
                lift.moveHigh();
                autoHighActive = true;
                autoHighClawOpened = false;
            }

            if ((raiseLift || lowerLift) && autoHighActive) {
                autoHighActive = false;
                autoHighClawOpened = false;
            }

            if (autoHighActive) {
                // PID control must update every loop while moving to/holding HIGH.
                lift.update();

                if (lift.atTarget() && !autoHighClawOpened) {
                    claw.open();
                    autoHighClawOpened = true;
                }
            } else {
                // Raw manual lift behavior from LiftTestTeleop.
                if (raiseLift && !lowerLift) {
                    lift.raise();
                } else if (lowerLift && !raiseLift) {
                    lift.lower();
                } else {
                    lift.stop();
                }
            }

            previousHighY = highYNow;

            // =========================================================
            // GAMEPAD 2 - CLAW CONTROL
            // While the automatic lift is still traveling, keep the claw closed.
            // After it opens at HIGH, the bumpers can override it normally.
            // =========================================================

            if (autoHighActive && !autoHighClawOpened) {
                claw.close();
            } else if (gamepad2.left_bumper) {
                claw.close();
            } else if (gamepad2.right_bumper) {
                claw.open();
            }

            // =========================================================
            // GAMEPAD 2 - 270 SERVO TEST CONTROL
            // Non-blocking version of Servo270Test.
            // =========================================================

            boolean servoANow = gamepad2.a;
            boolean servoAPressed = servoANow && !previousServoA;

            if (servoAPressed && !servo270CycleActive) {
                servo270OpenedTimeNs = System.nanoTime();
                servo270CycleActive = true;
            }

            if (servo270CycleActive) {
                double servoElapsedSeconds =
                        (System.nanoTime() - servo270OpenedTimeNs) * 1e-9;

                if (servoElapsedSeconds >= SERVO_270_HOLD_SECONDS) {
                    servo270CycleActive = false;
                }
            }

            // Reassert the selected command every loop. This makes the zero
            // position reliable and allows live Dashboard changes to take effect.
            servo270.setPosition(
                    servo270CycleActive
                            ? SERVO_270_OPEN_POSITION
                            : SERVO_270_ZERO_POSITION);

            previousServoA = servoANow;

            // =========================================================
            // UPDATE DRIVE LOCALIZATION
            // =========================================================

            Pose2d pose = drive.updateAndGetPose();

            // =========================================================
            // GAMEPAD 1 - RAW JOYSTICK INPUT
            // =========================================================

            double targetX = -gamepad1.left_stick_y;
            double targetY = -gamepad1.left_stick_x;
            double targetRotation = -gamepad1.right_stick_x;

            // =========================================================
            // DRIVE ACCELERATION LIMITING
            // =========================================================

            double maxDriveChange = DRIVE_ACCEL * loopPeriodSeconds;
            double maxRotationChange = ROTATION_ACCEL * loopPeriodSeconds;

            currentX += Math.max(
                    -maxDriveChange,
                    Math.min(maxDriveChange, targetX - currentX));

            currentY += Math.max(
                    -maxDriveChange,
                    Math.min(maxDriveChange, targetY - currentY));

            currentRotation += Math.max(
                    -maxRotationChange,
                    Math.min(maxRotationChange, targetRotation - currentRotation));

            // =========================================================
            // ROTATION / HEADING HOLD
            // =========================================================

            boolean manuallyRotating = Math.abs(targetRotation) > 0.05;

            if (manuallyRotating) {
                lastManualRotationTime = System.nanoTime();

                headingHold = false;
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            double timeSinceManualRotation =
                    (System.nanoTime() - lastManualRotationTime) * 1e-9;

            boolean rotationStopped = Math.abs(currentRotation) < 0.02;

            if (!manuallyRotating
                    && rotationStopped
                    && !headingHold
                    && timeSinceManualRotation >= HEADING_HOLD_DELAY) {

                headingHold = true;
                targetHeading = pose.heading.toDouble();
                headingIntegral = 0.0;
                previousHeadingError = 0.0;
            }

            // =========================================================
            // HEADING PID
            // =========================================================

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

                    double derivative =
                            (headingError - previousHeadingError) / dt;

                    headingCorrection =
                            headingKp * headingError
                                    + headingKi * headingIntegral
                                    + headingKd * derivative;

                    headingCorrection =
                            Math.max(-0.5, Math.min(0.5, headingCorrection));
                }

                previousHeadingError = headingError;
            }

            // =========================================================
            // DRIVE OUTPUT
            // =========================================================

            double speedMultiplier = slowMode ? 0.4 : 1.0;

            double rotationPower;

            if (manuallyRotating || !headingHold) {
                rotationPower = currentRotation * speedMultiplier;
            } else {
                rotationPower = headingCorrection;
            }

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

            telemetry.addLine("=== GAMEPAD 1 / DRIVE ===");
            telemetry.addData(
                    "Drive mode",
                    fieldCentric ? "Field-Centric" : "Robot-Centric");
            telemetry.addData("Slow Mode", slowMode);
            telemetry.addData("Pose X (in)", "%.2f", pose.position.x);
            telemetry.addData("Pose Y (in)", "%.2f", pose.position.y);
            telemetry.addData(
                    "Heading (deg)",
                    "%.1f",
                    Math.toDegrees(pose.heading.toDouble()));
            telemetry.addData(
                    "Target Heading (deg)",
                    "%.1f",
                    Math.toDegrees(targetHeading));
            telemetry.addData("Heading Hold", headingHold);
            telemetry.addData("Drive X", "%.2f", currentX);
            telemetry.addData("Drive Y", "%.2f", currentY);
            telemetry.addData("Rotation", "%.2f", currentRotation);

            telemetry.addLine("=== GAMEPAD 2 / SUBSYSTEMS ===");
            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Intake Encoder", intake.getCurrentPosition());
            telemetry.addData("Intake Velocity", "%.1f", intake.getVelocity());

            telemetry.addData("Lift Encoder", lift.getCurrentPosition());
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Error", lift.getError());
            telemetry.addData("Lift Height (%)", "%.1f", lift.getHeightFraction() * 100.0);
            telemetry.addData("Lift Homed", lift.isHomed());
            telemetry.addData("Magnetic Home Switch", lift.isHomeLimitPressed());
            telemetry.addData("Auto High", autoHighActive);
            telemetry.addData("High Claw Opened", autoHighClawOpened);

            if (highYPressed && !lift.isHomed()) {
                telemetry.addLine("GP2 Y IGNORED: home/calibrate lift first");
            }

            if (lowerLift && lift.isHomeLimitPressed()) {
                telemetry.addLine("LIFT LOWER BLOCKED: home switch is pressed");
            }

            telemetry.addData("Claw Position", "%.3f", claw.getPosition());
            telemetry.addData("270 Servo Position", "%.3f", servo270.getPosition());
            telemetry.addData("270 Servo Zero Setpoint", "%.3f", SERVO_270_ZERO_POSITION);
            telemetry.addData("270 Servo Open Setpoint", "%.3f", SERVO_270_OPEN_POSITION);
            telemetry.addData(
                    "270 Servo Cycle",
                    servo270CycleActive ? "OPEN / HOLDING" : "ZERO / HOLDING");

            telemetry.addData("Loop rate (Hz)", "%.1f", filteredLoopHz);
            telemetry.update();
        }

        intake.stop();
        lift.stop();
        claw.close();
        servo270.setPosition(SERVO_270_ZERO_POSITION);
        drive.stop();
    }
}
