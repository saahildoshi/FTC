package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.ShooterConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ShooterSubsystem;

/**
 * FTC DASHBOARD SHOOTER VELOCITY TUNER.
 *
 * Open FTC Dashboard -> Config -> ShooterDashboardTest while this OpMode is running.
 * Change the public static values below and they are copied into ShooterConstants
 * every loop so changes take effect immediately.
 *
 * Controls:
 * A = enable closed-loop shooter
 * B = stop shooter
 *
 * Recommended tuning order:
 * 1. Start with kP = kI = kD = 0 and tune kF until actual velocity is close to target.
 * 2. Increase kP until recovery from a shot is fast without strong oscillation.
 * 3. Add a small kD if the velocity overshoots or oscillates.
 * 4. Add only a very small kI if a repeatable steady-state error remains.
 */
@Config
@TeleOp(name = "Shooter Dashboard PID Test", group = "Testing")
public final class ShooterDashboardTest extends LinearOpMode {

    // ----------------------------------------------------------------------
    // LIVE DASHBOARD VALUES
    // ----------------------------------------------------------------------
    public static double kP = ShooterConstants.kP;
    public static double kI = ShooterConstants.kI;
    public static double kD = ShooterConstants.kD;
    public static double kF = ShooterConstants.kF;

    public static double TARGET_VELOCITY_TICKS_PER_SECOND =
            ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND;

    public static double MAX_TARGET_VELOCITY_TICKS_PER_SECOND =
            ShooterConstants.MAX_TARGET_VELOCITY_TICKS_PER_SECOND;

    public static double VELOCITY_TOLERANCE_TICKS_PER_SECOND =
            ShooterConstants.VELOCITY_TOLERANCE_TICKS_PER_SECOND;

    public static double MAX_POWER = ShooterConstants.MAX_POWER;
    public static double NOMINAL_VOLTAGE = ShooterConstants.NOMINAL_VOLTAGE;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initShooterHardware(hardwareMap);

        ShooterSubsystem shooter = new ShooterSubsystem(robot);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        boolean previousA = false;
        boolean previousB = false;

        telemetry.addLine("Shooter Dashboard PID Test Ready");
        telemetry.addLine("Open FTC Dashboard -> Config -> ShooterDashboardTest");
        telemetry.addLine("A = Start closed-loop shooter");
        telemetry.addLine("B = Stop shooter");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            shooter.stop();
            return;
        }

        while (opModeIsActive()) {

            // Push every Dashboard value into the real shooter constants.
            ShooterConstants.kP = kP;
            ShooterConstants.kI = kI;
            ShooterConstants.kD = kD;
            ShooterConstants.kF = kF;

            ShooterConstants.MAX_TARGET_VELOCITY_TICKS_PER_SECOND =
                    Math.max(0.0, MAX_TARGET_VELOCITY_TICKS_PER_SECOND);

            ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND =
                    Math.max(
                            0.0,
                            Math.min(
                                    Math.abs(TARGET_VELOCITY_TICKS_PER_SECOND),
                                    ShooterConstants.MAX_TARGET_VELOCITY_TICKS_PER_SECOND));

            ShooterConstants.VELOCITY_TOLERANCE_TICKS_PER_SECOND =
                    Math.max(0.0, VELOCITY_TOLERANCE_TICKS_PER_SECOND);

            ShooterConstants.MAX_POWER =
                    Math.max(0.0, Math.min(1.0, Math.abs(MAX_POWER)));

            ShooterConstants.NOMINAL_VOLTAGE =
                    Math.max(1.0, NOMINAL_VOLTAGE);

            boolean aNow = gamepad1.a;
            boolean bNow = gamepad1.b;

            if (aNow && !previousA) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            if (bNow && !previousB) {
                shooter.stop();
            }

            previousA = aNow;
            previousB = bNow;

            // Apply live target changes without stopping the shooter.
            if (shooter.isEnabled()) {
                shooter.setTargetVelocity(
                        ShooterConstants.TARGET_VELOCITY_TICKS_PER_SECOND);
            }

            shooter.update();

            telemetry.addLine("=== SHOOTER DASHBOARD TUNING ===");
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

            telemetry.addData("kP", "%.7f", kP);
            telemetry.addData("kI", "%.7f", kI);
            telemetry.addData("kD", "%.7f", kD);
            telemetry.addData("kF", "%.7f", kF);

            telemetry.addData(
                    "PID Output",
                    "%.4f",
                    shooter.getPidOutput());
            telemetry.addData(
                    "Feedforward Output",
                    "%.4f",
                    shooter.getFeedforwardOutput());
            telemetry.addData(
                    "Motor Power",
                    "%.4f",
                    shooter.getMotorPower());

            telemetry.addData(
                    "Battery Voltage",
                    "%.2f V",
                    shooter.getBatteryVoltage());
            telemetry.addData(
                    "Voltage Compensation",
                    "%.3f",
                    shooter.getVoltageCompensation());

            telemetry.addData(
                    "Tolerance (ticks/sec)",
                    "%.1f",
                    VELOCITY_TOLERANCE_TICKS_PER_SECOND);

            telemetry.update();
        }

        shooter.stop();
    }
}
