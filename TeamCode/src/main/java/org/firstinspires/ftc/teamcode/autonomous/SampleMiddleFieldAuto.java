package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.actions.LiftToPositionAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * ACTIVE SAMPLE ROAD RUNNER AUTONOMOUS
 *
 * PURPOSE:
 * This is a teaching/example autonomous centered at field coordinate (0, 0).
 * It demonstrates four important Road Runner concepts requested for the team:
 *
 *  1. splineTo()  - follow a smooth curved path.
 *  2. lineToX()   - drive along a straight line to a new X coordinate.
 *  3. strafeTo()  - translate directly to an X/Y point.
 *  4. ParallelAction / SequentialAction - coordinate mechanisms with each other.
 *
 * COORDINATE ASSUMPTION:
 * The robot starts at Pose2d(0, 0, 0), which means:
 *   X = 0 inches
 *   Y = 0 inches
 *   heading = 0 radians
 *
 * These coordinates are an artificial practice coordinate system, NOT a final
 * 2026-27 competition starting position. Replace them once the season field and
 * alliance starting locations are established.
 *
 * MECHANISM WARNING:
 * Lift encoder heights and claw servo positions are still starting values. Test
 * the mechanisms individually before running this complete autonomous.
 */
@Autonomous(name = "SAMPLE - Middle Field Road Runner Auto", group = "Samples")
public final class SampleMiddleFieldAuto extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------
        // STEP 1: DEFINE WHERE ROAD RUNNER THINKS THE ROBOT STARTS.
        // ------------------------------------------------------------------
        // Heading 0 means the robot is pointed along Road Runner's +X direction.
        Pose2d startPose = new Pose2d(0.0, 0.0, 0.0);

        // ------------------------------------------------------------------
        // STEP 2: MAP THE PHYSICAL ROBOT HARDWARE.
        // ------------------------------------------------------------------
        // RobotHardware connects Java names to Control Hub configuration names:
        // four drive motors, IMU, leftLift motor, and claw servo.
        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        // ------------------------------------------------------------------
        // STEP 3: CREATE SUBSYSTEMS.
        // ------------------------------------------------------------------
        // DriveSubsystem wraps the existing tuned Road Runner MecanumDrive.
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, startPose);

        // LiftSubsystem owns the active single encoder motor named "leftLift".
        LiftSubsystem lift = new LiftSubsystem(robot);

        // ClawSubsystem owns the active positional servo named "claw".
        ClawSubsystem claw = new ClawSubsystem(robot);

        // Start with the claw closed so the sample assumes the robot is holding
        // a game element before autonomous begins.
        claw.close();

        // ------------------------------------------------------------------
        // STEP 4: BUILD THE ROAD RUNNER DRIVE PATH.
        // ------------------------------------------------------------------
        // actionBuilder() creates a sequence of chassis movements beginning at
        // the start pose. Building the Action now does NOT move the robot yet.
        Action sampleDrivePath = drive.roadRunner()
                .actionBuilder(startPose)

                // SPLINE:
                // Travel from the origin to approximately (18, 18) using a smooth
                // curve. The final argument is the tangent direction Road Runner
                // should use as it reaches the end of the spline.
                .splineTo(new Vector2d(18.0, 18.0), Math.toRadians(45.0))

                // LINE TO:
                // Continue in a straight line until the robot's X coordinate is
                // 30 inches. This demonstrates Road Runner 1.0's line-style move.
                .lineToX(30.0)

                // STRAFE TO:
                // Move directly to field point (30, 6). Mecanum wheels allow the
                // chassis to translate sideways without first turning toward it.
                .strafeTo(new Vector2d(30.0, 6.0))

                // Finish constructing the drive Action. Again, build() only makes
                // the plan; Actions.runBlocking() below actually executes it.
                .build();

        // ------------------------------------------------------------------
        // STEP 5: CREATE MECHANISM ACTIONS.
        // ------------------------------------------------------------------
        // LiftToPositionAction stays alive until the encoder reaches HIGH_TICKS.
        Action raiseLiftHigh = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HIGH_TICKS
        );

        // SetClawAction sends the servo to its OPEN position and immediately ends.
        Action openClaw = new SetClawAction(claw, true);

        // A second lift Action will later return the mechanism to its base/home.
        Action lowerLiftHome = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HOME_TICKS
        );

        // ------------------------------------------------------------------
        // STEP 6: SHOW STATUS ON THE DRIVER STATION BEFORE START.
        // ------------------------------------------------------------------
        telemetry.addLine("Sample autonomous initialized.");
        telemetry.addLine("Start pose: (0, 0, 0 deg)");
        telemetry.addData("Lift target high", RobotConstants.Lift.HIGH_TICKS);
        telemetry.addData("Claw closed position", RobotConstants.Claw.CLOSED_POSITION);
        telemetry.addLine("WARNING: Verify lift/claw limits before physical test.");
        telemetry.update();

        // FTC remains here during INIT. Nothing in the Action sequence runs until
        // the field/Driver Station starts autonomous.
        waitForStart();

        // If STOP was pressed instead of START, leave without moving anything.
        if (isStopRequested()) {
            robot.stopLiftMotor();
            return;
        }

        // ------------------------------------------------------------------
        // STEP 7: EXECUTE THE COMPLETE AUTONOMOUS RECIPE.
        // ------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        // FIRST: execute the spline -> line -> strafe drive path.
                        sampleDrivePath,

                        // SECOND: ParallelAction starts BOTH child Actions together.
                        // The lift begins raising toward its highest preset while
                        // the claw receives its OPEN command at the same time.
                        //
                        // The claw Action finishes almost immediately, but the
                        // ParallelAction remains active until the lift also reaches
                        // its high encoder target.
                        new ParallelAction(
                                raiseLiftHigh,
                                openClaw
                        ),

                        // THIRD: because this is inside SequentialAction, lowering
                        // does not begin until the ParallelAction above is complete.
                        // The lift returns to HOME_TICKS/base after the claw opens.
                        lowerLiftHome
                )
        );

        // ------------------------------------------------------------------
        // STEP 8: SAFE END STATE.
        // ------------------------------------------------------------------
        // Road Runner's path has ended and the lift should be at home. Remove
        // commanded lift power as an extra cleanup step before the OpMode exits.
        lift.stop();
        drive.stop();
    }
}
