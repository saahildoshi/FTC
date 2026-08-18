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
import org.firstinspires.ftc.teamcode.drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

/**
 * LOOPING AUTO TEST
 *
 * This is intentionally a conservative diagnostic autonomous. It drives from
 * (60,36) to (0,0), operates the lift/claw, and then returns to (60,36).
 *
 * WHY THIS VERSION EXPLICITLY RESETS ROAD RUNNER LIMITS:
 * MecanumDrive is an @Config class, which means FTC Dashboard can modify its
 * static PARAMS values while the Robot Controller app is running. If either
 * maxWheelVel or maxAngVel becomes zero/negative, Road Runner can throw:
 *
 *     maxVels must be positive
 *
 * The source file may still show positive defaults even when the running Robot
 * Controller process contains a changed value. For this diagnostic test, we set
 * known-safe positive values BEFORE constructing DriveSubsystem/MecanumDrive.
 */
@Autonomous(name = "Looping Auto - TEST", group = "Samples")
public final class LoopingAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        // ------------------------------------------------------------------
        // STEP 1: FORCE KNOWN-VALID ROAD RUNNER MOTION CONSTRAINTS.
        // ------------------------------------------------------------------
        // These values are intentionally slower than the current tuned maximums.
        // They remove any zero/negative Dashboard-edited values from this test.
        MecanumDrive.PARAMS.maxWheelVel = 30.0;
        MecanumDrive.PARAMS.minProfileAccel = -20.0;
        MecanumDrive.PARAMS.maxProfileAccel = 20.0;
        MecanumDrive.PARAMS.maxAngVel = Math.PI;
        MecanumDrive.PARAMS.maxAngAccel = Math.PI;

        // ------------------------------------------------------------------
        // STEP 2: DEFINE START AND CENTER POSES.
        // ------------------------------------------------------------------
        Pose2d startPose = new Pose2d(
                60.0,
                36.0,
                Math.toRadians(180.0)
        );

        Pose2d centerPose = new Pose2d(
                0.0,
                0.0,
                Math.toRadians(180.0)
        );

        // ------------------------------------------------------------------
        // STEP 3: INITIALIZE HARDWARE AND SUBSYSTEMS.
        // ------------------------------------------------------------------
        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        // IMPORTANT: construct DriveSubsystem only AFTER restoring valid PARAMS.
        // MecanumDrive creates its default constraints during construction.
        DriveSubsystem drive = new DriveSubsystem(hardwareMap, startPose);
        LiftSubsystem lift = new LiftSubsystem(robot);
        ClawSubsystem claw = new ClawSubsystem(robot);
        claw.close();

        // ------------------------------------------------------------------
        // STEP 4: BUILD THE OUTBOUND PATH.
        // ------------------------------------------------------------------
        Action firstDrivePath;
        try {
            firstDrivePath = drive.roadRunner()
                    .actionBuilder(startPose)
                    .strafeTo(new Vector2d(0.0, 0.0))
                    .build();
        } catch (RuntimeException e) {
            telemetry.addLine("FAILED while building firstDrivePath");
            telemetry.addData("Error", e.getMessage());
            addConstraintTelemetry();
            telemetry.update();
            waitForStart();
            return;
        }

        // ------------------------------------------------------------------
        // STEP 5: BUILD THE RETURN PATH.
        // ------------------------------------------------------------------
        // For diagnosis this uses strafeTo instead of splineTo. If this version
        // works, we can safely reintroduce a spline afterward knowing the base
        // velocity constraints and pose transitions are valid.
        Action returnDrivePath;
        try {
            returnDrivePath = drive.roadRunner()
                    .actionBuilder(centerPose)
                    .strafeTo(new Vector2d(60.0, 36.0))
                    .build();
        } catch (RuntimeException e) {
            telemetry.addLine("FAILED while building returnDrivePath");
            telemetry.addData("Error", e.getMessage());
            addConstraintTelemetry();
            telemetry.update();
            waitForStart();
            return;
        }

        // ------------------------------------------------------------------
        // STEP 6: BUILD MECHANISM ACTIONS.
        // ------------------------------------------------------------------
        Action raiseLiftHigh = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HIGH_TICKS
        );

        Action openClaw = new SetClawAction(claw, true);
        Action closeClaw = new SetClawAction(claw, false);

        Action lowerLiftHome = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HOME_TICKS
        );

        // ------------------------------------------------------------------
        // STEP 7: SHOW LIVE CONSTRAINT VALUES BEFORE START.
        // ------------------------------------------------------------------
        telemetry.addLine("Looping autonomous initialized.");
        telemetry.addLine("Start pose: (60, 36, 180 deg)");
        telemetry.addLine("Center pose: (0, 0, 180 deg)");
        addConstraintTelemetry();
        telemetry.addData("Lift high", RobotConstants.Lift.HIGH_TICKS);
        telemetry.addData("Lift home", RobotConstants.Lift.HOME_TICKS);
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopLiftMotor();
            return;
        }

        // ------------------------------------------------------------------
        // STEP 8: RUN ONE OUT-AND-BACK CYCLE.
        // ------------------------------------------------------------------
        Actions.runBlocking(
                new SequentialAction(
                        new ParallelAction(
                                firstDrivePath,
                                openClaw
                        ),
                        closeClaw,
                        raiseLiftHigh,
                        new ParallelAction(
                                returnDrivePath,
                                lowerLiftHome
                        )
                )
        );

        lift.stop();
        drive.stop();
    }

    /**
     * Displays the exact values Road Runner is using at runtime. If the same
     * exception ever returns, these numbers tell us immediately whether a motion
     * limit has been changed to zero, negative, NaN, or infinity.
     */
    private void addConstraintTelemetry() {
        telemetry.addData("RR maxWheelVel", MecanumDrive.PARAMS.maxWheelVel);
        telemetry.addData("RR minProfileAccel", MecanumDrive.PARAMS.minProfileAccel);
        telemetry.addData("RR maxProfileAccel", MecanumDrive.PARAMS.maxProfileAccel);
        telemetry.addData("RR maxAngVel", MecanumDrive.PARAMS.maxAngVel);
        telemetry.addData("RR maxAngAccel", MecanumDrive.PARAMS.maxAngAccel);
        telemetry.addData("RR inPerTick", MecanumDrive.PARAMS.inPerTick);
        telemetry.addData("RR lateralInPerTick", MecanumDrive.PARAMS.lateralInPerTick);
        telemetry.addData("RR trackWidthTicks", MecanumDrive.PARAMS.trackWidthTicks);
    }
}
