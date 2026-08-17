package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.actions.LiftToPositionAction;
import org.firstinspires.ftc.teamcode.actions.SetClawAction;
import org.firstinspires.ftc.teamcode.config.RobotConstants;
import org.firstinspires.ftc.teamcode.hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystems.ClawSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.DriveSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.LiftSubsystem;

@Autonomous(name = "Looping Auto - TEST", group = "Samples")
public final class LoopingAutoTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d startPose = new Pose2d(60, 36, 180);

        RobotHardware robot = new RobotHardware();
        robot.initVerifiedHardware(hardwareMap);

        DriveSubsystem drive = new DriveSubsystem(hardwareMap, startPose);
        LiftSubsystem lift = new LiftSubsystem(robot);
        ClawSubsystem claw = new ClawSubsystem(robot);
        claw.close();

        Action firstDrivePath = drive.roadRunner()
                .actionBuilder(startPose)

                .strafeTo(new Vector2d(0,0))
                .build();

        Action returnDrivePath = drive.roadRunner()
                .actionBuilder(startPose)
                .splineTo(new Vector2d(60, 36), Math.toRadians(0))
                .build();

        Action raiseLiftHigh = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HIGH_TICKS
        );

        Action openClaw = new SetClawAction(claw, true);
        Action closeClaw = new SetClawAction(claw, false);

        Action lowerLiftHome = new LiftToPositionAction(
                lift,
                RobotConstants.Lift.HIGH_TICKS
        );

        telemetry.addLine("Sample autonomous initialized.");
        telemetry.addLine("Start pose: (60, 36, 180 deg)");
        telemetry.addData("Lift target high", RobotConstants.Lift.HIGH_TICKS);
        telemetry.addData("Claw closed position", RobotConstants.Claw.CLOSED_POSITION);
        telemetry.addLine("WARNING: Verify lift/claw limits before physical test.");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) {
            robot.stopLiftMotor();
            return;
        }
        while (opModeIsActive()) {
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
        }
    }
}
