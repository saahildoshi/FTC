package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Simple servo test.
 *
 * Press A:
 *   1. Servo moves to 180-degree position.
 *   2. Holds for 2 seconds.
 *   3. Automatically returns to the starting position.
 */
@TeleOp(name = "270 Degree Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final String SERVO_NAME = "servo270";

    // For a 270-degree servo:
    // 0.0 = approximately 0 degrees
    // 2/3 = approximately 180 degrees
    private static final double CLOSED_POSITION = 0.0;
    private static final double OPEN_POSITION = 2.0 / 3.0;

    private static final double HOLD_TIME_SECONDS = 2.0;

    @Override
    public void runOpMode() throws InterruptedException {

        Servo servo270 = hardwareMap.get(Servo.class, SERVO_NAME);

        // Keep reversed direction to match the current mechanism setup.
        servo270.setDirection(Servo.Direction.REVERSE);

        ElapsedTime timer = new ElapsedTime();

        boolean previousA = false;
        boolean isOpen = false;

        servo270.setPosition(CLOSED_POSITION);

        telemetry.addLine("Servo Test Ready");
        telemetry.addLine("Press A to open 180 degrees for 2 seconds");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.a && !previousA;

            if (aPressed && !isOpen) {
                servo270.setPosition(OPEN_POSITION);
                timer.reset();
                isOpen = true;
            }

            if (isOpen && timer.seconds() >= HOLD_TIME_SECONDS) {
                servo270.setPosition(CLOSED_POSITION);
                isOpen = false;
            }

            telemetry.addData("Servo Position", "%.3f", servo270.getPosition());
            telemetry.addData("State", isOpen ? "OPEN" : "CLOSED");

            if (isOpen) {
                telemetry.addData(
                        "Reset In",
                        "%.2f sec",
                        Math.max(0.0, HOLD_TIME_SECONDS - timer.seconds()));
            }

            telemetry.update();

            previousA = gamepad1.a;
            idle();
        }

        servo270.setPosition(CLOSED_POSITION);
    }
}
