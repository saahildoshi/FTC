package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * Standalone 270-degree positional servo test.
 *
 * This test intentionally DOES NOT initialize RobotHardware. That keeps an
 * unrelated missing motor/sensor from preventing the servo test from running.
 *
 * Controls:
 *   A = step 0 -> 90 -> 180 -> 270, hold 3 sec, return to 0
 *   X = force 0 degrees
 *   Y = force 90 degrees
 *   B = force 180 degrees
 *   Right bumper = force 270 degrees
 */
@TeleOp(name = "270 Degree Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final String SERVO_NAME = "servo270";

    private static final double ZERO_DEGREES = 0.0;
    private static final double NINETY_DEGREES = 1.0 / 3.0;
    private static final double ONE_EIGHTY_DEGREES = 2.0 / 3.0;
    private static final double TWO_SEVENTY_DEGREES = 1.0;

    private static final double RESET_DELAY_SECONDS = 3.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Only initialize the device being tested.
        Servo servo270 = hardwareMap.get(Servo.class, SERVO_NAME);

        // Keep the direction reversed to match the mechanism configuration.
        servo270.setDirection(Servo.Direction.REVERSE);

        int pressCount = 0;
        boolean previousA = false;
        boolean waitingToReset = false;
        double commandedPosition = ZERO_DEGREES;

        ElapsedTime resetTimer = new ElapsedTime();

        servo270.setPosition(commandedPosition);

        telemetry.addLine("270 Degree Servo Test Ready");
        telemetry.addData("Hardware name", SERVO_NAME);
        telemetry.addLine("A: step 90 -> 180 -> 270 -> 3 sec -> 0");
        telemetry.addLine("X=0, Y=90, B=180, RB=270");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.a && !previousA;

            // Direct-position buttons make it easy to prove each endpoint.
            if (gamepad1.x) {
                commandedPosition = ZERO_DEGREES;
                pressCount = 0;
                waitingToReset = false;
            }
            else if (gamepad1.y) {
                commandedPosition = NINETY_DEGREES;
                pressCount = 1;
                waitingToReset = false;
            }
            else if (gamepad1.b) {
                commandedPosition = ONE_EIGHTY_DEGREES;
                pressCount = 2;
                waitingToReset = false;
            }
            else if (gamepad1.right_bumper) {
                commandedPosition = TWO_SEVENTY_DEGREES;
                pressCount = 3;
                waitingToReset = false;
            }
            else if (aPressed && !waitingToReset) {
                pressCount++;

                if (pressCount == 1) {
                    commandedPosition = NINETY_DEGREES;
                }
                else if (pressCount == 2) {
                    commandedPosition = ONE_EIGHTY_DEGREES;
                }
                else if (pressCount >= 3) {
                    commandedPosition = TWO_SEVENTY_DEGREES;
                    pressCount = 3;
                    resetTimer.reset();
                    waitingToReset = true;
                }
            }

            if (waitingToReset && resetTimer.seconds() >= RESET_DELAY_SECONDS) {
                commandedPosition = ZERO_DEGREES;
                pressCount = 0;
                waitingToReset = false;
            }

            servo270.setPosition(commandedPosition);

            telemetry.addData("Press Count", pressCount);
            telemetry.addData("Commanded Position", "%.3f", commandedPosition);
            telemetry.addData("SDK Servo Position", "%.3f", servo270.getPosition());
            telemetry.addData("Direction", servo270.getDirection());
            telemetry.addData("Waiting To Reset", waitingToReset);

            if (waitingToReset) {
                telemetry.addData(
                        "Seconds Until Reset",
                        "%.2f",
                        Math.max(0.0, RESET_DELAY_SECONDS - resetTimer.seconds()));
            }

            telemetry.update();

            previousA = gamepad1.a;
            idle();
        }

        servo270.setPosition(ZERO_DEGREES);
    }
}
