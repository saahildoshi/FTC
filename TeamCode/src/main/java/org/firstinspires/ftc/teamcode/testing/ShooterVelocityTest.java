package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * SHOOTER VELOCITY PID TEST
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
 * Watch Target Velocity, Actual Velocity, Velocity Error, PID Output,
 * Feedforward Output, and Ready To Shoot in Driver Station / FTC Dashboard.
 *
 * A useful physical test is to let the flywheel stabilize, feed balls through it,
 * and watch how quickly Actual Velocity returns to Target Velocity after each shot.
 */
@TeleOp(name = "Shooter Velocity PID Test", group = "Testing")
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

        telemetry.addLine("Shooter Velocity PID Test Ready");
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

            if (gamepad1.a) {
                shooter.start();
            }

            if (gamepad1.b) {
                shooter.stop();
            }

            // If Dashboard or D-pad changes the target while running, apply it live.
            if (shooter.isEnabled()) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            shooter.update();

            telemetry.addLine("=== SHOOTER VELOCITY CONTROL ===");
            telemetry.addData("Enabled", shooter.isEnabled());
            telemetry.addData("Ready To Shoot", shooter.isReadyToShoot());
            telemetry.addData(
                    "Target Velocity (ticks/sec)",
                    "%.1f",
                    shooter.getTargetVelocity());
            telemetry.addData(
                    "Actual Velocity (ticks/sec)",
                    "%.1f",
                    shooter.getCurrentVelocity());
            telemetry.addData(
                    "Velocity Error (ticks/sec)",
                    "%.1f",
                    shooter.getVelocityError());

            telemetry.addData("kP", ShooterConstants.kP);
            telemetry.addData("kI", ShooterConstants.kI);
            telemetry.addData("kD", ShooterConstants.kD);
            telemetry.addData("kF", ShooterConstants.kF);

            telemetry.addData("PID Output", "%.4f", shooter.getPidOutput());
            telemetry.addData(
                    "Feedforward Output",
                    "%.4f",
                    shooter.getFeedforwardOutput());
            telemetry.addData(
                    "Voltage Compensation",
                    "%.3f",
                    shooter.getVoltageCompensation());
            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    shooter.getBatteryVoltage());
            telemetry.addData(
                    "Motor Power",
                    "%.3f",
                    shooter.getMotorPower());

            telemetry.update();
        }

        shooter.stop();
    }
}
