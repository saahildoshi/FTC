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

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.FlowerDescorerSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.GateSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

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
 * - Right trigger: toggle intake forward
 * - Left trigger: toggle intake reverse
 * - A: start the PIDF shooter and automatic gate
 * - B: stop the shooter and close the gate
 * - Y: raise/stow the flower descorer
 * - X: lower/deploy the flower descorer
 *
 * The gate opens automatically at the ShooterConstants upper velocity threshold
 * and closes below its lower threshold, matching ShooterGateAutomaticTest.
 */
@Config
@TeleOp(name = "Mecanum Drive (Road Runner 2.0)", group = "TeleOp")
public final class TeleOpDrive extends LinearOpMode {

    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0));

    // Dashboard-adjustable intake power, matching IntakeMotorTest behavior.
    public static double INTAKE_TEST_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // =========================================================
        // HARDWARE / SUBSYSTEMS
        // =========================================================

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, INITIAL_POSE);

        RobotHardware robot = new RobotHardware();
        robot.initIntakeHardware(hardwareMap);
        robot.initShooterHardware(hardwareMap);
        robot.initGateHardware(hardwareMap);
        robot.initFlowerDescorerHardware(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(robot);
        ShooterSubsystem shooter = new ShooterSubsystem(robot);
        GateSubsystem gate = new GateSubsystem(robot);
        FlowerDescorerSubsystem flowerDescorer =
                new FlowerDescorerSubsystem(robot);

        gate.close();
        flowerDescorer.zero();

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

        boolean previousShooterStart = false;
        boolean previousShooterStop = false;
        boolean previousFlowerUp = false;
        boolean previousFlowerDown = false;
        boolean previousIntakeForward = false;
        boolean previousIntakeReverse = false;
        int intakeDirection = 0;

        // ---------------- Loop Timing ----------------

        long previousLoopTimeNs;
        double filteredLoopHz = 0.0;

        telemetry.addLine("TeleOp initialized");
        telemetry.addLine("GAMEPAD 1: drive only");
        telemetry.addLine("GAMEPAD 2: mechanisms only");
        telemetry.addLine("GP2 RT/LT: toggle intake forward/reverse");
        telemetry.addLine("GP2 A/B: start/stop PIDF shooter");
        telemetry.addLine("GP2 Y/X: flower descorer raise/lower");
        telemetry.addLine("Shooter gate is automatic");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            intake.stop();
            shooter.stop();
            gate.close();
            flowerDescorer.zero();
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
            // GAMEPAD 2 - INTAKE TOGGLE CONTROL
            // =========================================================

            boolean intakeForwardNow = gamepad2.right_trigger > 0.10;
            boolean intakeReverseNow = gamepad2.left_trigger > 0.10;
            boolean intakeForwardPressed =
                    intakeForwardNow && !previousIntakeForward;
            boolean intakeReversePressed =
                    intakeReverseNow && !previousIntakeReverse;

            if (intakeForwardPressed && intakeReversePressed) {
                intakeDirection = 0;
            } else if (intakeForwardPressed) {
                intakeDirection = intakeDirection == 1 ? 0 : 1;
            } else if (intakeReversePressed) {
                intakeDirection = intakeDirection == -1 ? 0 : -1;
            }

            previousIntakeForward = intakeForwardNow;
            previousIntakeReverse = intakeReverseNow;

            intake.setPower(intakeDirection * Math.abs(INTAKE_TEST_POWER));

            // =========================================================
            // GAMEPAD 2 - PIDF SHOOTER / AUTOMATIC GATE
            // =========================================================

            boolean shooterStartNow = gamepad2.a;
            boolean shooterStopNow = gamepad2.b;

            if (shooterStartNow && !previousShooterStart) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            if (shooterStopNow && !previousShooterStop) {
                shooter.stop();
                gate.close();
            }

            previousShooterStart = shooterStartNow;
            previousShooterStop = shooterStopNow;

            // Reapply the live Dashboard target without resetting PIDF unless
            // the configured target actually changes.
            if (shooter.isEnabled()) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            shooter.update();

            double shooterVelocity = shooter.getCurrentVelocity();
            double gateOpenVelocity = Math.max(
                    0.0,
                    ShooterConstants.GATE_OPEN_VELOCITY_TICKS_PER_SECOND);
            double gateCloseVelocity = Math.min(
                    gateOpenVelocity,
                    Math.max(
                            0.0,
                            ShooterConstants.GATE_CLOSE_VELOCITY_TICKS_PER_SECOND));

            if (!shooter.isEnabled()) {
                gate.close();
            } else if (gate.isOpen()) {
                if (shooterVelocity < gateCloseVelocity) {
                    gate.close();
                }
            } else if (shooterVelocity >= gateOpenVelocity) {
                gate.open();
            }

            // =========================================================
            // GAMEPAD 2 - FLOWER DESCORER
            // =========================================================

            boolean flowerUpNow = gamepad2.y;
            boolean flowerDownNow = gamepad2.x;

            if (flowerUpNow && !previousFlowerUp) {
                flowerDescorer.zero();
            }

            if (flowerDownNow && !previousFlowerDown) {
                flowerDescorer.lower();
            }

            previousFlowerUp = flowerUpNow;
            previousFlowerDown = flowerDownNow;

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
            telemetry.addData(
                    "Intake State",
                    intakeDirection > 0
                            ? "FORWARD"
                            : intakeDirection < 0 ? "REVERSE" : "OFF");
            telemetry.addData("Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Intake Encoder", intake.getCurrentPosition());
            telemetry.addData("Intake Velocity", "%.1f", intake.getVelocity());

            telemetry.addData("Shooter Enabled", shooter.isEnabled());
            telemetry.addData(
                    "Shooter Target",
                    "%.1f ticks/sec",
                    shooter.getTargetVelocity());
            telemetry.addData(
                    "Shooter Velocity",
                    "%.1f ticks/sec",
                    shooterVelocity);
            telemetry.addData(
                    "Shooter Error",
                    "%.1f ticks/sec",
                    shooter.getVelocityError());
            telemetry.addData("Shooter Ready", shooter.isReadyToShoot());
            telemetry.addData("Shooter PID", "%.3f", shooter.getPidOutput());
            telemetry.addData("Shooter Feedforward", "%.3f", shooter.getFeedforwardOutput());
            telemetry.addData("Shooter Power", "%.3f", shooter.getMotorPower());
            telemetry.addData("Gate State", gate.isOpen() ? "OPEN" : "CLOSED");
            telemetry.addData(
                    "Gate Thresholds",
                    "open %.0f / close %.0f ticks/sec",
                    gateOpenVelocity,
                    gateCloseVelocity);
            telemetry.addData(
                    "Flower Descorer",
                    flowerDescorer.isLowered() ? "LOWERED" : "RAISED");
            telemetry.addData(
                    "Flower Positions",
                    "left %.3f / right %.3f",
                    flowerDescorer.getLeftPosition(),
                    flowerDescorer.getRightPosition());

            telemetry.addData("Loop rate (Hz)", "%.1f", filteredLoopHz);
            telemetry.update();
        }

        intake.stop();
        shooter.stop();
        gate.close();
        flowerDescorer.zero();
        drive.stop();
    }
}
