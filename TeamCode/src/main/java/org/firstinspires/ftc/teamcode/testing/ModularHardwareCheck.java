package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;

/**
 * VERIFIED HARDWARE CHECK
 *
 * Run this before drive testing after changing the Robot Controller hardware
 * configuration. If init succeeds, Java found all four verified drive motors
 * and the IMU using the expected configuration names.
 *
 * This test intentionally does NOT move anything. Its job is simply to verify
 * that software names match the physical devices configured on the Control Hub.
 */
@TeleOp(name = "TEST - Modular Hardware Check", group = "Testing")
public final class ModularHardwareCheck extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();

        try {
            robot.initVerifiedHardware(hardwareMap);
            telemetry.addLine("PASS: verified drivetrain hardware and IMU found.");
        } catch (Exception exception) {
            telemetry.addLine("FAIL: a configured device could not be found.");
            telemetry.addData("Error", exception.getMessage());
        }

        telemetry.addLine("No motors will move in this test.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addLine("Hardware check complete.");
            telemetry.update();
            sleep(250);
        }

        robot.stopDriveMotors();
    }
}
