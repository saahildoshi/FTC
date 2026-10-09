package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * SHOOTER VELOCITY PIDF TEST
 *
 * Robot Configuration name:
 * shooter
 *
 * Controls:
 * A = start closed-loop shooter
 * B = stop shooter
 * D-pad Up = target velocity +50 ticks/sec
 * D-pad Down = target velocity -50 ticks/sec
 *
 * Watch Target Velocity, Actual Velocity, Velocity Error, PID Feedback,
 * Feedforward, PIDF Command, and Shooter State in Driver Station / Dashboard.
 *
 * A useful physical test is to let the flywheel stabilize, feed balls through it,
 * and watch how quickly Actual Velocity returns to Target Velocity after each shot.
 */
@TeleOp(name = "Shooter Velocity PIDF Test", group = "Testing")
public final class ShooterVelocityTest extends LinearOpMode {

    private static final double VELOCITY_STEP = 50.0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initShooterHardware(hardwareMap);

        ShooterSubsystem shooter = new ShooterSubsystem(robot);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        boolean previousDpadUp = false;
        boolean previousDpadDown = false;
        boolean previousA = false;
        boolean previousB = false;

        telemetry.addLine("Shooter Velocity PIDF Test Ready");
        telemetry.addLine("A = Start shooter");
        telemetry.addLine("B = Stop shooter");
        telemetry.addLine("D-pad Up/Down = +/- 50 ticks/sec");
        telemetry.addData(
                "Target Velocity (ticks/sec)",
                ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            shooter.stop();
            return;
        }

        while (opModeIsActive()) {

            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;
            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;

            if (dpadUp && !previousDpadUp) {
                ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND =
                        Math.min(
                                ShooterConstants.MAX_TARGET_VELOCITY_TICKS_PER_SECOND,
                                ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND
                                        + VELOCITY_STEP);
            }

            if (dpadDown && !previousDpadDown) {
                ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND =
                        Math.max(
                                0.0,
                                ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND
                                        - VELOCITY_STEP);
            }

            previousDpadUp = dpadUp;
            previousDpadDown = dpadDown;

            if (aNow && !previousA) {
                shooter.start();
            }

            if (bNow && !previousB) {
                shooter.stop();
            }

            previousA = aNow;
            previousB = bNow;

            // If Dashboard or D-pad changes the target while running, apply it live.
            if (shooter.isEnabled()) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            shooter.update();

            double targetVelocity = shooter.getTargetVelocity();
            double currentVelocity = shooter.getCurrentVelocity();
            double velocityError = targetVelocity - currentVelocity;
            double pidFeedback = shooter.getPidOutput();
            double feedforward = shooter.getFeedforwardOutput();
            double pidfCommand = pidFeedback + feedforward;
            double motorPower = shooter.getMotorPower();
            double maxPower = Math.abs(ShooterConstants.MAX_POWER);

            telemetry.addLine("=== SHOOTER PIDF VELOCITY CONTROL ===");
            telemetry.addData("Enabled", shooter.isEnabled());
            telemetry.addData(
                    "Shooter State",
                    getShooterState(shooter, velocityError));
            telemetry.addData(
                    "Target Velocity (ticks/sec)",
                    "%.1f",
                    targetVelocity);
            telemetry.addData(
                    "Actual Velocity (ticks/sec)",
                    "%.1f",
                    currentVelocity);
            telemetry.addData(
                    "Velocity Error (ticks/sec)",
                    "%.1f",
                    velocityError);

            telemetry.addData("kP", ShooterConstants.kP);
            telemetry.addData("kI", ShooterConstants.kI);
            telemetry.addData("kD", ShooterConstants.kD);
            telemetry.addData("kF", ShooterConstants.kF);

            telemetry.addData("PID Feedback", "%.4f", pidFeedback);
            telemetry.addData(
                    "Feedforward",
                    "%.4f",
                    feedforward);
            telemetry.addData(
                    "PIDF Command (before voltage)",
                    "%.4f",
                    pidfCommand);
            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    shooter.getBatteryVoltage());
            telemetry.addData(
                    "Motor Power",
                    "%.3f",
                    motorPower);
            telemetry.addData(
                    "Power Saturated",
                    maxPower > 0.0 && motorPower >= maxPower - 0.0001);

            telemetry.update();
        }

        shooter.stop();
    }

    private static String getShooterState(
            ShooterSubsystem shooter,
            double velocityError) {
        if (!shooter.isEnabled()) {
            return "STOPPED";
        }

        double tolerance =
                Math.abs(ShooterConstants.VELOCITY_TOLERANCE_TICKS_PER_SECOND);
        if (Math.abs(velocityError) <= tolerance) {
            return "READY";
        }

        return velocityError > 0.0 ? "RECOVERING" : "OVERSPEED";
    }
}
