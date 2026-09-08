package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * SIMPLE 270 SERVO TEST
 *
 * Press A once:
 * - move the servo to approximately 180 degrees
 * - hold for 2 seconds
 * - automatically return to the start position
 */
@TeleOp(name = "270 Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final double START_POSITION = 0.0;
    private static final double OPEN_180_POSITION = 2.0 / 3.0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo270 = hardwareMap.get(Servo.class, "servo270");

        servo270.setDirection(Servo.Direction.REVERSE);
        servo270.setPosition(START_POSITION);

        telemetry.addLine("270 Servo Test Ready");
        telemetry.addLine("Press A: open to 180 degrees, wait 2 sec, reset");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                servo270.setPosition(OPEN_180_POSITION);
                telemetry.addLine("OPEN - 180 degrees");
                telemetry.update();

                sleep(2000);

                servo270.setPosition(START_POSITION);
                telemetry.addLine("RESET - 0 degrees");
                telemetry.update();

                // Prevent one long button hold from immediately triggering again.
                while (opModeIsActive() && gamepad1.a) {
                    idle();
                }
            }

            telemetry.addData("Servo Position", servo270.getPosition());
            telemetry.update();
            idle();
        }

        servo270.setPosition(START_POSITION);
    }
}
