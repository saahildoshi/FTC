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

@Config
@TeleOp(name = "Mecanum Drive (Road Runner 1.0)", group = "TeleOp")
public final class TeleOpDrive extends LinearOpMode {

    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0));

    private static final double INTAKE_POWER = 0.50;

    public static double SERVO_270_START_POSITION = 0.0;
    public static double SERVO_270_OPEN_180_POSITION = 2.0 / 3.0;
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

        // Direction is configured here, but the servo is intentionally NOT moved
        // during init. This keeps TeleOp startup from creating unnecessary servo load.
        servo270.setDirection(Servo.Direction.REVERSE);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        // =========================================================
        // GAMEPAD 1 - DRIVE STATE
        // =========================================================

        boolean slowMode = false;
        boolean previousA = false;

        boolean fieldCentric = false;
        boolean previousY = false;
        boolean previousHeadingReset = false;
        boolean previousX = false;

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

        long previousLoopTimeNs;
        double filteredLoopHz = 0.0;

        telemetry.addLine("TeleOp initialized");
        telemetry.addLine("GAMEPAD 1: drivetrain");
        telemetry.addLine("GAMEPAD 2: mechanisms");
        telemetry.addLine("GP2 RT or X: intake forward");
        telemetry.addLine("GP2 LT or B: intake reverse");
        telemetry.addLine("GP2 D-pad Up/Down: lift manual");
        telemetry.addLine("GP2 Y: lift HIGH -> open claw");
        telemetry.addLine("GP2 LB/RB: claw close/open");
        telemetry.addLine("GP2 A: 270 servo 180 deg -> 1 sec -> reset");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            intake.stop();
            lift.stop();
            drive.stop();
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
                filteredLoopHz = filteredLoopHz == 0.0
                        ? instantaneousLoopHz
                        : 0.90 * filteredLoopHz + 0.10 * instantaneousLoopHz;
            }

            // =========================================================
            // GAMEPAD 1 - DRIVE BUTTONS
            // =========================================================

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

            boolean xPressed = gamepad1.x;
            if (xPressed && !previousX) {
                telemetry.addLine("Returning drivetrain home...");
                telemetry.update();

                Pose2d currentPose = drive.updateAndGetPose();

                Action returnHome = drive.roadRunner()
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
            // GAMEPAD 2 - INTAKE
            // =========================================================
            // X/B are digital diagnostic controls. The triggers remain enabled.
            // Digital buttons get priority so they completely bypass trigger input.

            double commandedIntakePower = 0.0;
            String intakeCommandSource = "STOP";

            if (gamepad2.x) {
                commandedIntakePower = Math.abs(INTAKE_POWER);
                intakeCommandSource = "X FORWARD";
            } else if (gamepad2.b) {
                commandedIntakePower = -Math.abs(INTAKE_POWER);
                intakeCommandSource = "B REVERSE";
            } else if (gamepad2.right_trigger > 0.10) {
                commandedIntakePower = Math.abs(INTAKE_POWER);
                intakeCommandSource = "RIGHT TRIGGER";
            } else if (gamepad2.left_trigger > 0.10) {
                commandedIntakePower = -Math.abs(INTAKE_POWER);
                intakeCommandSource = "LEFT TRIGGER";
            }

            intake.setPower(commandedIntakePower);

            // =========================================================
            // GAMEPAD 2 - AUTOMATIC HIGH LIFT
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
                lift.update();

                if (lift.atTarget() && !autoHighClawOpened) {
                    claw.open();
                    autoHighClawOpened = true;
                }
            } else {
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
            // GAMEPAD 2 - CLAW
            // =========================================================

            if (autoHighActive && !autoHighClawOpened) {
                claw.close();
            } else if (gamepad2.left_bumper) {
                claw.close();
            } else if (gamepad2.right_bumper) {
                claw.open();
            }

            // =========================================================
            // GAMEPAD 2 - 270 SERVO
            // =========================================================

            boolean servoANow = gamepad2.a;
            boolean servoAPressed = servoANow && !previousServoA;

            if (servoAPressed && !servo270CycleActive) {
                servo270.setPosition(SERVO_270_OPEN_180_POSITION);
                servo270OpenedTimeNs = System.nanoTime();
                servo270CycleActive = true;
            }

            if (servo270CycleActive) {
                double servoElapsedSeconds =
                        (System.nanoTime() - servo270OpenedTimeNs) * 1e-9;

                if (servoElapsedSeconds >= SERVO_270_HOLD_SECONDS) {
                    servo270.setPosition(SERVO_270_START_POSITION);
                    servo270CycleActive = false;
                }
            }

            previousServoA = servoANow;

            // =========================================================
            // DRIVE LOCALIZATION / INPUT
            // =========================================================

            Pose2d pose = drive.updateAndGetPose();

            double targetX = -gamepad1.left_stick_y;
            double targetY = -gamepad1.left_stick_x;
            double targetRotation = -gamepad1.right_stick_x;

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
            // HEADING HOLD
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

            double rotationPower = (manuallyRotating || !headingHold)
                    ? currentRotation * speedMultiplier
                    : headingCorrection;

            double outputX = currentX * speedMultiplier;
            double outputY = currentY * speedMultiplier;

            if (fieldCentric) {
                drive.fieldCentric(outputX, outputY, rotationPower);
            } else {
                drive.robotCentric(outputX, outputY, rotationPower);
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
            telemetry.addData("Heading Hold", headingHold);

            telemetry.addLine("=== GAMEPAD 2 / INTAKE ===");
            telemetry.addData("GP2 Right Trigger", "%.3f", gamepad2.right_trigger);
            telemetry.addData("GP2 Left Trigger", "%.3f", gamepad2.left_trigger);
            telemetry.addData("GP2 X", gamepad2.x);
            telemetry.addData("GP2 B", gamepad2.b);
            telemetry.addData("Intake Source", intakeCommandSource);
            telemetry.addData("Commanded Intake Power", "%.2f", commandedIntakePower);
            telemetry.addData("Actual Intake Motor Power", "%.2f", intake.getPower());
            telemetry.addData("Intake Encoder", intake.getCurrentPosition());
            telemetry.addData("Intake Velocity", "%.1f", intake.getVelocity());
            telemetry.addData("Battery Voltage", "%.2f V", robot.voltageSensor.getVoltage());

            telemetry.addLine("=== GAMEPAD 2 / OTHER SUBSYSTEMS ===");
            telemetry.addData("Lift Encoder", lift.getCurrentPosition());
            telemetry.addData("Lift Target", lift.getTargetPosition());
            telemetry.addData("Lift Homed", lift.isHomed());
            telemetry.addData("Magnetic Home Switch", lift.isHomeLimitPressed());
            telemetry.addData("Auto High", autoHighActive);
            telemetry.addData("Claw Position", "%.3f", claw.getPosition());
            telemetry.addData("270 Servo Position", "%.3f", servo270.getPosition());
            telemetry.addData(
                    "270 Servo Cycle",
                    servo270CycleActive ? "OPEN / HOLDING" : "READY");

            if (highYPressed && !lift.isHomed()) {
                telemetry.addLine("GP2 Y IGNORED: home/calibrate lift first");
            }

            if (lowerLift && lift.isHomeLimitPressed()) {
                telemetry.addLine("LIFT LOWER BLOCKED: home switch is pressed");
            }

            telemetry.addData("Loop rate (Hz)", "%.1f", filteredLoopHz);
            telemetry.update();
        }

        intake.stop();
        lift.stop();
        drive.stop();
    }
}
