package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * 270 DEGREE POSITIONAL SERVO STEP TEST.
 *
 * A button sequence:
 * 1st press = 90 degrees
 * 2nd press = 180 degrees
 * 3rd press = 270 degrees
 * After the 3rd press, hold at 270 degrees for 3 seconds,
 * then automatically return to 0 degrees and reset the press count.
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

        // Servo direction is intentionally reversed for this mechanism.
        servo270.setDirection(Servo.Direction.REVERSE);

        int pressCount = 0;
        boolean previousA = false;
        boolean waitingToReset = false;

        ElapsedTime resetTimer = new ElapsedTime();

        // Start at the zero position.
        servo270.setPosition(ZERO_DEGREES);

        telemetry.addLine("270 Degree Servo Step Test Ready");
        telemetry.addLine("Press A three times: 90 -> 180 -> 270 -> wait 3 sec -> 0");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Count only a new press, not a held button.
            boolean aPressed = gamepad1.a && !previousA;

            if (aPressed && !waitingToReset) {
                pressCount++;

                if (pressCount == 1) {
                    servo270.setPosition(NINETY_DEGREES);
                }
                else if (pressCount == 2) {
                    servo270.setPosition(ONE_EIGHTY_DEGREES);
                }
                else if (pressCount == 3) {
                    // Third press goes to the full 270-degree position first.
                    servo270.setPosition(TWO_SEVENTY_DEGREES);

                    // Start a non-blocking three-second hold timer.
                    resetTimer.reset();
                    waitingToReset = true;
                }
            }

            // Hold at 270 degrees for three seconds before resetting.
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
            }
            else if (pressCount == 1) {
                telemetry.addData("Approx Angle", "90 degrees");
            }
            else if (pressCount == 2) {
                telemetry.addData("Approx Angle", "180 degrees");
            }
            else {
                telemetry.addData("Approx Angle", "270 degrees");
                telemetry.addData(
                        "Seconds Until Reset",
                        Math.max(0.0, RESET_DELAY_SECONDS - resetTimer.seconds()));
            }

            telemetry.update();

            previousA = gamepad1.a;
        }

        servo270.setPosition(ZERO_DEGREES);
    }
}
