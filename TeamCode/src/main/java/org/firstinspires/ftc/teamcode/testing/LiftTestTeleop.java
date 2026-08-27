package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * MANUAL SWYFT LIFT HARDWARE TEST.
 *
 * This OpMode intentionally does NOT call lift.update(). The purpose is to test
 * raw manual motor direction and the REV magnetic home switch without the PID
 * controller fighting the driver's command.
 *
 * Controls (either controller):
 * - D-pad Up: raise lift
 * - D-pad Down: lower lift
 *
 * Downward motion is still blocked when the magnetic home switch is pressed.
 */
@TeleOp(name = "Test - Lift", group = "Testing")
public class LiftTestTeleop extends LinearOpMode {

    @Override
    public void runOpMode() {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(robot);

        telemetry = new MultipleTelemetry(
                telemetry,
                FtcDashboard.getInstance().getTelemetry());

        telemetry.addLine("SWYFT Lift Manual Test Ready");
        telemetry.addLine("D-pad Up = Raise");
        telemetry.addLine("D-pad Down = Lower");
        telemetry.addLine("Either controller can be used");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {

            boolean raisePressed =
                    gamepad1.dpad_up || gamepad2.dpad_up;

            boolean lowerPressed =
                    gamepad1.dpad_down || gamepad2.dpad_down;

            // IMPORTANT:
            // Do not call lift.update() here. This is a direct manual motor test.
            if (raisePressed && !lowerPressed) {
                lift.raise();
            } else if (lowerPressed && !raisePressed) {
                lift.lower();
            } else {
                lift.stop();
            }

            telemetry.addData("Raise Command", raisePressed);
            telemetry.addData("Lower Command", lowerPressed);
            telemetry.addData("Encoder Ticks", lift.getCurrentPosition());
            telemetry.addData("Lift Height (ticks)", lift.getHeightTicks());
            telemetry.addData("Lift Height (%)", "%.1f", lift.getHeightFraction() * 100.0);
            telemetry.addData("Lift Homed", lift.isHomed());
            telemetry.addData("Magnetic Limit", lift.isHomeLimitPressed());

            if (lowerPressed && lift.isHomeLimitPressed()) {
                telemetry.addLine("LOWER BLOCKED: magnetic home switch is PRESSED");
            }

            telemetry.update();
        }

        lift.stop();
    }
}
