package org.firstinspires.ftc.teamcode.testing;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.IntakeSubsystem;

/**
 * DIAGNOSTIC: drivetrain + intake only.
 *
 * This intentionally does NOT create the lift, claw, or 270 servo subsystem.
 * If the intake works here but not in TeleOpDrive, another mechanism command is
 * interfering. If it still fails here, investigate drivetrain hardware mapping
 * or a shared motor-controller port in the Robot Controller configuration.
 */
@TeleOp(name = "TEST - Drive + Intake Isolation", group = "Testing")
public final class DriveIntakeIsolationTest extends LinearOpMode {

    private static final Pose2d INITIAL_POSE = new Pose2d(
            new Vector2d(0.0, 0.0),
            Rotation2d.exp(0.0));

    private static final double INTAKE_POWER = 0.50;

    @Override
    public void runOpMode() throws InterruptedException {

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, INITIAL_POSE);

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);
        IntakeSubsystem intake = new IntakeSubsystem(robot);
        intake.stop();

        telemetry.addLine("Drive + Intake Isolation Test Ready");
        telemetry.addLine("GP1 left stick = drive / strafe");
        telemetry.addLine("GP1 right stick X = rotate");
        telemetry.addLine("GP2 X or RT = intake forward");
        telemetry.addLine("GP2 B or LT = intake reverse");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            intake.stop();
            drive.stop();
            return;
        }

        while (opModeIsActive()) {

            double forward = -gamepad1.left_stick_y;
            double strafe = -gamepad1.left_stick_x;
            double turn = -gamepad1.right_stick_x;
            drive.robotCentric(forward, strafe, turn);

            double intakePower = 0.0;

            if (gamepad2.x || gamepad2.right_trigger > 0.10) {
                intakePower = INTAKE_POWER;
            } else if (gamepad2.b || gamepad2.left_trigger > 0.10) {
                intakePower = -INTAKE_POWER;
            }

            intake.setPower(intakePower);

            telemetry.addData("GP2 X", gamepad2.x);
            telemetry.addData("GP2 B", gamepad2.b);
            telemetry.addData("GP2 RT", "%.3f", gamepad2.right_trigger);
            telemetry.addData("GP2 LT", "%.3f", gamepad2.left_trigger);
            telemetry.addData("Commanded Intake Power", "%.2f", intakePower);
            telemetry.addData("Actual Intake Power", "%.2f", intake.getPower());
            telemetry.addData("Intake Velocity", "%.1f", intake.getVelocity());
            telemetry.addData("Battery Voltage", "%.2f V", robot.voltageSensor.getVoltage());
            telemetry.update();
        }

        intake.stop();
        drive.stop();
    }
}
