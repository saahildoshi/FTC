package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp(name = "Test - Lift", group = "Testing")
public class LiftTestTeleop extends LinearOpMode {
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        LiftSubsystem lift = new LiftSubsystem(robot);
        telemetry.addLine("Lift Test Ready");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 1 Up = Raise");
        telemetry.addLine("Gamepad 1 Down = Lower");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        while (opModeIsActive()) {
            lift.update();

            if (gamepad1.dpad_up) {
                lift.raise();
            } else if (gamepad1.dpad_down) {
                lift.lower();
            } else {
                lift.stop();
            }

            telemetry.addData("Lift Height (ticks)", lift.getHeightTicks());
            telemetry.addData("Lift Height (%)", "%.1f", lift.getHeightFraction() * 100.0);
            telemetry.addData("Lift Homed", lift.isHomed());
            telemetry.addData("Magnetic Limit", lift.isHomeLimitPressed());
            telemetry.update();
        }

        lift.stop();
    }
}
