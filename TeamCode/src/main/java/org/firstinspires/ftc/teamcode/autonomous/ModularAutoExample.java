package org.firstinspires.ftc.teamcode.autonomous;

/**
 * MODULAR AUTONOMOUS INTEGRATION GUIDE
 *
 * Autonomous should describe WHAT the robot does, while subsystems/actions
 * contain HOW each part of the robot performs the work.
 *
 * TYPICAL FLOW:
 *
 * Autonomous OpMode
 *   -> DriveSubsystem
 *      -> Road Runner trajectory Action
 *         -> mecanum drivetrain
 *
 *   -> future Lift/Arm/Intake/Claw Actions
 *      -> mechanism subsystems
 *         -> motors/servos
 *
 * Example structure after the team's starting pose and mechanism choices are
 * known:
 *
 * --------------------------------------------------------------------------
 *
 * @Autonomous(name = "Modular Auto")
 * public class ModularAuto extends LinearOpMode {
 *     public void runOpMode() {
 *         Pose2d startPose = new Pose2d(0, 0, 0); // REPLACE with real field pose.
 *         DriveSubsystem drive = new DriveSubsystem(hardwareMap, startPose);
 *
 *         Action driveToScore = drive.roadRunner()
 *                 .actionBuilder(startPose)
 *                 // Add the season-specific trajectory here.
 *                 .build();
 *
 *         waitForStart();
 *         if (isStopRequested()) return;
 *
 *         Actions.runBlocking(new SequentialAction(
 *                 driveToScore,
 *                 new StopDriveAction(drive)
 *                 // Future examples:
 *                 // new LiftToPositionAction(lift, HIGH),
 *                 // new OpenClawAction(claw),
 *                 // new IntakeAction(intake)
 *         ));
 *     }
 * }
 *
 * --------------------------------------------------------------------------
 *
 * Road Runner also supports ParallelAction. That lets the robot drive and move
 * a mechanism at the same time, which can save autonomous time once each action
 * is individually reliable.
 *
 * This stays inactive until the 2026-27 field/start pose and real mechanisms are
 * established, preventing a placeholder autonomous from appearing on the Driver
 * Station as if it were competition-ready.
 */
public final class ModularAutoExample {
    private ModularAutoExample() { }
}
