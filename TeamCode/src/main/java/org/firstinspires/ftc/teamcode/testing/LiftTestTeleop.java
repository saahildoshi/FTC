package org.firstinspires.ftc.teamcode.testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@TeleOp(name = "Test - Lift", group = "Testing")
public class LiftTestTeleop extends LinearOpMode{
    @Override
    public void runOpMode() {
        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        LiftSubsystem liftMotor = new LiftSubsystem(robot);
        telemetry.addLine("Lift Test Ready");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 1 Up = Raise");
        telemetry.addLine("Gamepad 1 Down = Lower");
        telemetry.update();

        waitForStart();
//        if (isStopRequested()) {
//            return;
//        }
        while(opModeIsActive()) {
            if (gamepad1.dpad_up) {
                liftMotor.raise();
            }
            if (gamepad1.dpad_down) {
                liftMotor.lower();
            }
//            else{
//                liftMotor.stop();
//            }
        }
        telemetry.addLine("Lift Test");
        telemetry.addLine("");
        telemetry.addLine("Gamepad 1 UP = Raise");
        telemetry.addLine("Gamepad 1 DOWN = Lower");

        telemetry.addData(
                "Lift Position",
                liftMotor.getCurrentPosition()
        );

        telemetry.update();
    }
}
