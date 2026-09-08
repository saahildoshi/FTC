package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * 270 DEGREE POSITIONAL SERVO STEP TEST.
 *
 * Control:
 * - Each NEW press of A advances the servo by 1/3 of its range.
 * - Press 1 -> 90 degrees
 * - Press 2 -> 180 degrees
 * - Press 3 -> 270 degrees
 * - Hold at 270 degrees for 3 seconds, then reset to 0 degrees and reset the count.
 */
@TeleOp(name = "270 Degree Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final double ZERO_DEGREES = 0.0;
    private static final double NINETY_DEGREES = 1.0 / 3.0;
    private static final double ONE_EIGHTY_DEGREES = 2.0 / 3.0;
    private static final double TWO_SEVENTY_DEGREES = 1.0;
    private static final double RESET_DELAY_SECONDS = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        Servo servo270 = robot.servo270;

        // Keep the reversed physical direction from the previous test.
        servo270.setDirection(Servo.Direction.REVERSE);

        int pressCount = 0;
        boolean previousA = false;
        boolean waitingToReset = false;
        ElapsedTime resetTimer = new ElapsedTime();

        // Start at 0 degrees.
        servo270.setPosition(ZERO_DEGREES);

        telemetry.addLine("270 Degree Servo Step Test Ready");
        telemetry.addLine("A = advance one-third");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Rising-edge detection: holding A only counts once.
            boolean aPressed = gamepad1.a && !previousA;

            if (aPressed && !waitingToReset) {
                pressCount++;

                if (pressCount == 1) {
                    servo270.setPosition(NINETY_DEGREES);
                } else if (pressCount == 2) {
                    servo270.setPosition(ONE_EIGHTY_DEGREES);
                } else if (pressCount == 3) {
                    servo270.setPosition(TWO_SEVENTY_DEGREES);
                    resetTimer.reset();
                    waitingToReset = true;
                }
            }

            // After 3 seconds at 270 degrees, return to zero and restart the sequence.
            if (waitingToReset && resetTimer.seconds() >= RESET_DELAY_SECONDS) {
                servo270.setPosition(ZERO_DEGREES);
                pressCount = 0;
                waitingToReset = false;
            }

            telemetry.addData("Press Count", pressCount);
            telemetry.addData("Servo Position", servo270.getPosition());
            telemetry.addData("Direction", "REVERSE");
            telemetry.addData("Waiting To Reset", waitingToReset);

            if (pressCount == 0) {
                telemetry.addData("Approx Angle", "0 degrees");
            } else if (pressCount == 1) {
                telemetry.addData("Approx Angle", "90 degrees");
            } else if (pressCount == 2) {
                telemetry.addData("Approx Angle", "180 degrees");
            } else {
                telemetry.addData("Approx Angle", "270 degrees");
                telemetry.addData("Reset In", Math.max(0.0,
                        RESET_DELAY_SECONDS - resetTimer.seconds()));
            }

            telemetry.update();

            previousA = gamepad1.a;
        }

        servo270.setPosition(ZERO_DEGREES);
    }
}
