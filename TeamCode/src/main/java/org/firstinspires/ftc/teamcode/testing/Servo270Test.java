package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * SIMPLE POSITION TEST FOR A 270 DEGREE POSITIONAL SERVO.
 *
 * Controls:
 * - A = 0 degrees
 * - X = 90 degrees
 * - Y = 180 degrees
 * - B = 270 degrees
 */
@TeleOp(name = "270 Degree Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final double ZERO_DEGREES = 0.0;
    private static final double NINETY_DEGREES = 1.0 / 3.0;
    private static final double ONE_EIGHTY_DEGREES = 2.0 / 3.0;
    private static final double TWO_SEVENTY_DEGREES = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        Servo servo270 = robot.servo270;

        // If the physical direction is backwards, uncomment this line:
        // servo270.setDirection(Servo.Direction.REVERSE);

        // Start at 0 degrees.
        servo270.setPosition(ZERO_DEGREES);

        telemetry.addLine("270 Degree Servo Test Ready");
        telemetry.addLine("A = 0 degrees");
        telemetry.addLine("X = 90 degrees");
        telemetry.addLine("Y = 180 degrees");
        telemetry.addLine("B = 270 degrees");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo270.setPosition(ZERO_DEGREES);
            }

            if (gamepad1.x) {
                servo270.setPosition(NINETY_DEGREES);
            }

            if (gamepad1.y) {
                servo270.setPosition(ONE_EIGHTY_DEGREES);
            }

            if (gamepad1.b) {
                servo270.setPosition(TWO_SEVENTY_DEGREES);
            }

            telemetry.addData("Servo Position", servo270.getPosition());
            telemetry.addData("0 deg", ZERO_DEGREES);
            telemetry.addData("90 deg", NINETY_DEGREES);
            telemetry.addData("180 deg", ONE_EIGHTY_DEGREES);
            telemetry.addData("270 deg", TWO_SEVENTY_DEGREES);
            telemetry.update();
        }
    }
}
