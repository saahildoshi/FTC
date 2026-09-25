package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * Adjustable intake motor test.
 *
 * Controls:
 * - Right trigger: forward
 * - Left trigger: reverse
 *
 * Forward and reverse always use the exact same TEST_POWER magnitude.
 * TEST_POWER can be changed live from FTC Dashboard while this OpMode is running.
 */
@Config
@TeleOp(name = "Intake Motor Test", group = "Testing")
public final class IntakeMotorTest extends LinearOpMode {

    public static double TEST_POWER = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        IntakeSubsystem intake = new IntakeSubsystem(robot);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("Intake Motor Test Ready");
        telemetry.addLine("Right trigger = Forward");
        telemetry.addLine("Left trigger = Reverse");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            intake.stop();
            return;
        }

        while (opModeIsActive()) {

            double commandedPower = 0.0;

            if (gamepad1.right_trigger > 0.10) {
                commandedPower = Math.abs(TEST_POWER);
            } else if (gamepad1.left_trigger > 0.10) {
                commandedPower = -Math.abs(TEST_POWER);
            }

            intake.setPower(commandedPower);

            telemetry.addData("Test Power", TEST_POWER);
            telemetry.addData("Commanded Power", commandedPower);
            telemetry.addData("Actual Motor Power", intake.getPower());
            telemetry.addData("Encoder Position", intake.getCurrentPosition());
            telemetry.addData("Velocity (ticks/sec)", intake.getVelocity());
            telemetry.addData("Absolute Velocity", Math.abs(intake.getVelocity()));
            telemetry.addData("Battery Voltage", "%.2f V", robot.voltageSensor.getVoltage());
            telemetry.addData("Intake connection", robot.intake.getConnectionInfo());
            telemetry.addData("Configured motor type", robot.intake.getMotorType().getName());
            telemetry.update();
        }

        intake.stop();
    }
}
