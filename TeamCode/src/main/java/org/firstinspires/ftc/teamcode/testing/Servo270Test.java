package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * TEST FOR A 270 DEGREE POSITIONAL SERVO.
 *
 * Controls:
 * - Press A once: 0 -> 90 degrees
 * - Press A again: 90 -> 180 degrees
 * - Press A a third time: 180 -> 270 degrees
 * - After a short delay at 270 degrees, the servo returns to 0 degrees
 */
@TeleOp(name = "270 Degree Servo Test", group = "Testing")
public final class Servo270Test extends LinearOpMode {

    private static final double ZERO_POSITION = 0.0;
    private static final double STEP_1_POSITION = 1.0 / 3.0;
    private static final double STEP_2_POSITION = 2.0 / 3.0;
    private static final double STEP_3_POSITION = 1.0;

    private static final double RESET_DELAY_SECONDS = 0.75;

    @Override
    public void runOpMode() throws InterruptedException {

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        Servo servo = robot.servo270;

        // If the physical left/right direction is reversed, uncomment this line:
        // servo.setDirection(Servo.Direction.REVERSE);

        int step = 0;
        boolean previousA = false;
        boolean waitingToReset = false;

        ElapsedTime resetTimer = new ElapsedTime();

        servo.setPosition(ZERO_POSITION);

        telemetry.addLine("270 Degree Servo Test Ready");
        telemetry.addLine("A = Move Left 90 Degrees");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            boolean aPressed = gamepad1.a && !previousA;

            if (aPressed && !waitingToReset) {
                step++;

                switch (step) {
                    case 1:
                        servo.setPosition(STEP_1_POSITION);
                        break;

                    case 2:
                        servo.setPosition(STEP_2_POSITION);
                        break;

                    case 3:
                        servo.setPosition(STEP_3_POSITION);
                        resetTimer.reset();
                        waitingToReset = true;
                        break;

                    default:
                        step = 0;
                        servo.setPosition(ZERO_POSITION);
                        break;
                }
            }

            if (waitingToReset
                    && resetTimer.seconds() >= RESET_DELAY_SECONDS) {
                servo.setPosition(ZERO_POSITION);
                step = 0;
                waitingToReset = false;
            }

            telemetry.addData("Step", step);
            telemetry.addData("Servo Position", servo.getPosition());

            if (step == 0) {
                telemetry.addData("Approx Angle", "0 degrees");
            } else if (step == 1) {
                telemetry.addData("Approx Angle", "90 degrees");
            } else if (step == 2) {
                telemetry.addData("Approx Angle", "180 degrees");
            } else {
                telemetry.addData("Approx Angle", "270 degrees");
            }

            telemetry.addData("Waiting To Reset", waitingToReset);
            telemetry.update();

            previousA = gamepad1.a;
        }

        servo.setPosition(ZERO_POSITION);
    }
}
