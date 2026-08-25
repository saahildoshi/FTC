package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
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
 * LOOPING AUTO TEST
 *
 * This autonomous demonstrates a trip from the starting location to the center
 * of the field and then back to the original starting location while operating
 * the lift and claw.
 *
 * IMPORTANT ROAD RUNNER RULE:
 * Every Action must be built from the pose where that Action actually begins.
 * A zero-length path (starting and ending at the same point) can cause Road
 * Runner to calculate a zero maximum path velocity and throw:
 *
 *     maxVels must be positive
 *
 * The previous returnDrivePath was built from startPose (60,36) and also ended
 * at (60,36), creating exactly that zero-length trajectory. This version builds
 * the return path from centerPose (0,0), where the first drive path actually ends.
 */
@Autonomous(name = "Looping Auto - TEST", group = "Samples")
public final class LoopingAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------
        // DEFINE THE TWO IMPORTANT FIELD POSES.
        // ------------------------------------------------------------------

        // Pose headings are ALWAYS in radians in Road Runner.
        // Math.toRadians(180) correctly represents a 180-degree heading.
        Pose2d startPose = new Pose2d(
                60.0,
                36.0,
                Math.toRadians(180.0)
        );

        // strafeTo() keeps the robot's current heading, so after driving from
        // startPose to (0,0), the robot is still facing approximately 180 degrees.
        // This is therefore the correct starting pose for returnDrivePath.
        Pose2d centerPose = new Pose2d(
                0.0,
                0.0,
                Math.toRadians(180.0)
        );
        Pose2d otherPose = new Pose2d(
                -60,
                0,
                Math.toRadians(180)
        );

        // ------------------------------------------------------------------
        // INITIALIZE ROBOT HARDWARE AND SUBSYSTEMS.
        // ------------------------------------------------------------------
        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, startPose);
        LiftSubsystem lift = new LiftSubsystem(robot);
        ClawSubsystem claw = new ClawSubsystem(robot);

        // Begin with the claw closed.
        claw.close();

        // ------------------------------------------------------------------
        // OUTBOUND DRIVE PATH: (60,36) -> (0,0)
        // ------------------------------------------------------------------
        Action firstDrivePath = drive.roadRunner()
                .actionBuilder(startPose)

                // Mecanum strafe directly to the field center while preserving
                // the robot's current heading.
                .strafeTo(new Vector2d(0.0, 0.0))
                .stopAndAdd(new SleepAction(1))
                .build();

        Action secondDrivePath = drive.roadRunner()
                .actionBuilder(centerPose)

                .strafeTo(new Vector2d(-60, -36))
                .stopAndAdd(new SleepAction(1))
                .build();

        Action thirdDrivePath = drive.roadRunner()
                .actionBuilder(startPose)

                // Mecanum strafe directly to the field center while preserving
                // the robot's current heading.
                .lineToY(0)
                .stopAndAdd(new SleepAction(1))
                .build();
        // ------------------------------------------------------------------
        // RETURN DRIVE PATH: (0,0) -> (60,36)
        // ------------------------------------------------------------------
        // CRITICAL FIX:
        // This Action must begin at centerPose because that is where
        // firstDrivePath ends. Building it from startPose would make the spline
        // begin and end at (60,36), creating a zero-length path.
        Action returnDrivePath = drive.roadRunner()
                .actionBuilder(centerPose)

                // Smoothly curve from the center back to the original starting
                // coordinates. The final tangent points along +X (0 degrees).
                .strafeTo(
                        new Vector2d(60.0, 36.0)
                )
                .stopAndAdd(new SleepAction(1))
                .build();
        Action returnTwoDrivePath = drive.roadRunner()
                .actionBuilder(otherPose)

                // Smoothly curve from the center back to the original starting
                // coordinates. The final tangent points along +X (0 degrees).
                .strafeTo(
                        new Vector2d(60.0, 36.0)
                )
                .stopAndAdd(new SleepAction(1))
                .build();

        // ------------------------------------------------------------------
        // MECHANISM ACTIONS
        // ------------------------------------------------------------------
        Action raiseLiftHigh = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HIGH_TICKS
        );

        Action openClaw = new SetClawAction(claw, true);
        Action closeClaw = new SetClawAction(claw, false);

        // FIX: lowering the lift must target HOME_TICKS, not HIGH_TICKS.
        Action lowerLiftHome = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HOME_TICKS
        );

        // ------------------------------------------------------------------
        // DRIVER STATION INFORMATION DURING INIT
        // ------------------------------------------------------------------
        telemetry.addLine("Looping autonomous initialized.");
        telemetry.addLine("Start pose: (60, 36, 180 deg)");
        telemetry.addLine("Center pose: (0, 0, 180 deg)");
        telemetry.addData("Lift target high", RobotConstants.Lift.HIGH_TICKS);
        telemetry.addData("Lift target home", RobotConstants.Lift.HOME_TICKS);
        telemetry.addData("Claw closed position", RobotConstants.Claw.CLOSED_POSITION);
        telemetry.addLine("WARNING: Verify lift/claw limits before physical test.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopLiftMotor();
            return;
        }

        // ------------------------------------------------------------------
        // RUN ONE COMPLETE OUT-AND-BACK CYCLE.
        // ------------------------------------------------------------------
            Actions.runBlocking(
                    new SequentialAction(

                            // Drive to the center while opening the claw.
                            new ParallelAction(
                                    firstDrivePath
                            ),

                            // Close the claw once the center position is reached.
                            //closeClaw,

                            // Raise the lift to its high preset.
                            //raiseLiftHigh,

                            // Return toward the starting position while lowering the
                            // lift back to its base/home encoder position.
                            new ParallelAction(
                                    secondDrivePath,
                                    raiseLiftHigh
                            ),
                            openClaw,
                            new ParallelAction(
                                    thirdDrivePath,
                                    closeClaw,
                                    lowerLiftHome
                            ),
                            returnTwoDrivePath
                    )

            );

        // Remove mechanism/chassis commands when the test finishes.
        lift.stop();
        drive.stop();
    }
}
