package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.roadrunner.AccelConstraint;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.TranslationalVelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.VelConstraint;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
@Autonomous(name = "Square Drive Test (Road Runner 1.0)", group = "Test")
public final class SquareTestAuto extends LinearOpMode {
    private static final double SIDE_LENGTH_IN = 24.0;
    private static final Pose2d START_POSE = new Pose2d(0.0, 0.0, 0.0);

    // Conservative initial test limits. Increase only after repeatable testing.
    private static final VelConstraint TEST_VEL = new TranslationalVelConstraint(30.0);
    private static final AccelConstraint TEST_ACCEL =
            new ProfileAccelConstraint(-20.0, 20.0);

    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap, START_POSE);

        Action square = drive.actionBuilder(START_POSE)
                .strafeToConstantHeading(
                        new Vector2d(SIDE_LENGTH_IN, 0.0), TEST_VEL, TEST_ACCEL)
                .waitSeconds(0.5)
                .strafeToConstantHeading(
                        new Vector2d(SIDE_LENGTH_IN, SIDE_LENGTH_IN), TEST_VEL, TEST_ACCEL)
                .waitSeconds(0.5)
                .strafeToConstantHeading(
                        new Vector2d(0.0, SIDE_LENGTH_IN), TEST_VEL, TEST_ACCEL)
                .waitSeconds(0.5)
                .strafeToConstantHeading(
                        new Vector2d(0.0, 0.0), TEST_VEL, TEST_ACCEL)
                .build();

        telemetry.addLine("Place the robot at the marked (0, 0) start pose.");
        telemetry.addData("Path", "24 in counterclockwise square");
        telemetry.addData("Heading", "Constant 0 deg");
        telemetry.addData("Test speed", "30 in/s max, 20 in/s^2 max");
        telemetry.update();

        waitForStart();
        if (isStopRequested()) {
            return;
        }

        Actions.runBlocking(square);

        Pose2d finalPose = drive.localizer.getPose();
        telemetry.addLine("Square complete");
        telemetry.addData("Final X (in)", "%.2f", finalPose.position.x);
        telemetry.addData("Final Y (in)", "%.2f", finalPose.position.y);
        telemetry.addData("Final heading (deg)", "%.1f",
                Math.toDegrees(finalPose.heading.toDouble()));
        telemetry.update();

        while (opModeIsActive()) {
            idle();
        }
    }
}
