package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;


@Config
@TeleOp(name = "Generic Drive TeleOp", group = "TeleOp")
public final class GenericDriveTeleOp extends LinearOpMode {

    /*
     * Maximum change in normalized driver command per second.
     *
     * 4.0 means:
     * 0 -> 1 takes 0.25 seconds minimum.
     */
    public static double INPUT_RAMP_RATE = 4.0;

    public static double SLOW_FRACTION = 0.40;

    public static double DEADBAND = 0.05;


    @Override
    public void runOpMode() {

        // =========================================================
        // DRIVE
        // =========================================================

        MecanumDrive drive =
                new MecanumDrive(
                        hardwareMap,
                        new Pose2d(0.0, 0.0, 0.0)
                );


        telemetry =
                new MultipleTelemetry(
                        telemetry,
                        FtcDashboard.getInstance().getTelemetry()
                );


        // =========================================================
        // MODES
        // =========================================================

        boolean slowMode = false;
        boolean fieldCentric = false;

        boolean previousA = false;
        boolean previousY = false;
        boolean previousX = false;
        boolean previousHeadingReset = false;


        // =========================================================
        // INITIAL ROAD RUNNER POSE
        // =========================================================

        drive.updatePoseEstimate();

        Pose2d actualPose =
                drive.localizer.getPose();


        // =========================================================
        // MOVING TARGET
        // =========================================================

        double targetX =
                actualPose.position.x;

        double targetY =
                actualPose.position.y;

        double targetHeading =
                actualPose.heading.toDouble();


        // Current target velocity.
        double targetVx = 0.0;
        double targetVy = 0.0;
        double targetOmega = 0.0;


        // =========================================================
        // RAMPED DRIVER INPUT
        // =========================================================

        double rampedForward = 0.0;
        double rampedLeft = 0.0;
        double rampedTurn = 0.0;


        telemetry.addLine("Drive initialized");
        telemetry.addLine("Left stick: drive / strafe");
        telemetry.addLine("Right stick X: rotate target heading");
        telemetry.addLine("A: slow mode");
        telemetry.addLine("Y: field / robot centric");
        telemetry.addLine("X: return to (0, 0, 0)");
        telemetry.addLine("OPTIONS / START: reset heading");
        telemetry.update();


        waitForStart();

        if (isStopRequested()) {
            stopDrive(drive);
            return;
        }


        long previousTime =
                System.nanoTime();


        while (opModeIsActive()) {

            // =====================================================
            // TIME
            // =====================================================

            long now =
                    System.nanoTime();

            double dt =
                    (now - previousTime) * 1e-9;

            previousTime = now;


            if (dt <= 0.0 || dt > 0.1) {
                dt = 0.02;
            }


            // =====================================================
            // UPDATE LOCALIZATION
            // =====================================================

            PoseVelocity2d actualVelocity =
                    drive.updatePoseEstimate();

            actualPose =
                    drive.localizer.getPose();


            // =====================================================
            // BUTTONS
            // =====================================================

            boolean aPressed =
                    gamepad1.a;

            boolean yPressed =
                    gamepad1.y;

            boolean xPressed =
                    gamepad1.x;

            boolean headingResetPressed =
                    gamepad1.options
                            || gamepad1.start;


            // Slow mode
            if (aPressed && !previousA) {
                slowMode = !slowMode;
            }


            // Field / robot centric
            if (yPressed && !previousY) {
                fieldCentric = !fieldCentric;
            }


            // =====================================================
            // HEADING RESET
            // =====================================================

            if (headingResetPressed
                    && !previousHeadingReset) {

                drive.resetHeading();

                actualPose =
                        drive.localizer.getPose();


                // Target must agree with the newly reset pose.
                targetX =
                        actualPose.position.x;

                targetY =
                        actualPose.position.y;

                targetHeading =
                        actualPose.heading.toDouble();


                targetVx = 0.0;
                targetVy = 0.0;
                targetOmega = 0.0;

                rampedForward = 0.0;
                rampedLeft = 0.0;
                rampedTurn = 0.0;


                actualVelocity =
                        new PoseVelocity2d(
                                new Vector2d(0.0, 0.0),
                                0.0
                        );
            }


            // =====================================================
            // RETURN TO (0, 0, 0)
            // =====================================================

            if (xPressed && !previousX) {

                Action returnHome =
                        drive.actionBuilder(
                                        drive.localizer.getPose()
                                )
                                .strafeToLinearHeading(
                                        new Vector2d(0.0, 0.0),
                                        0.0
                                )
                                .build();


                Actions.runBlocking(returnHome);


                drive.updatePoseEstimate();

                actualPose =
                        drive.localizer.getPose();


                // The moving TeleOp target must now start
                // from wherever the trajectory finished.
                targetX =
                        actualPose.position.x;

                targetY =
                        actualPose.position.y;

                targetHeading =
                        actualPose.heading.toDouble();


                targetVx = 0.0;
                targetVy = 0.0;
                targetOmega = 0.0;

                rampedForward = 0.0;
                rampedLeft = 0.0;
                rampedTurn = 0.0;
            }


            previousA = aPressed;
            previousY = yPressed;
            previousX = xPressed;
            previousHeadingReset = headingResetPressed;


            // =====================================================
            // RAW DRIVER INPUT
            // =====================================================

            double forwardInput =
                    deadband(
                            -gamepad1.left_stick_y
                    );

            double leftInput =
                    deadband(
                            -gamepad1.left_stick_x
                    );

            double turnInput =
                    deadband(
                            -gamepad1.right_stick_x
                    );


            // =====================================================
            // SLOW MODE
            // =====================================================

            double speedFraction =
                    slowMode
                            ? SLOW_FRACTION
                            : 1.0;


            /*
             * Apply slow mode BEFORE the ramp.
             *
             * That means even changing into/out of slow mode cannot
             * instantly jump the requested drive command.
             */
            double desiredForward =
                    forwardInput * speedFraction;

            double desiredLeft =
                    leftInput * speedFraction;

            double desiredTurn =
                    turnInput * speedFraction;


            // =====================================================
            // HARD INPUT RAMP LIMIT
            // =====================================================

            double maximumInputChange =
                    INPUT_RAMP_RATE * dt;


            rampedForward =
                    moveToward(
                            rampedForward,
                            desiredForward,
                            maximumInputChange
                    );


            rampedLeft =
                    moveToward(
                            rampedLeft,
                            desiredLeft,
                            maximumInputChange
                    );


            rampedTurn =
                    moveToward(
                            rampedTurn,
                            desiredTurn,
                            maximumInputChange
                    );


            // =====================================================
            // TARGET VELOCITY
            // =====================================================

            double oldVx = targetVx;
            double oldVy = targetVy;
            double oldOmega = targetOmega;


            double maxLinearVelocity =
                    MecanumDrive.PARAMS.maxWheelVel;

            double maxAngularVelocity =
                    MecanumDrive.PARAMS.maxAngVel;


            if (fieldCentric) {

                targetVx =
                        rampedForward
                                * maxLinearVelocity;

                targetVy =
                        rampedLeft
                                * maxLinearVelocity;

            } else {

                /*
                 * Convert robot-relative driver direction
                 * into Road Runner world coordinates.
                 */
                double robotForward =
                        rampedForward
                                * maxLinearVelocity;

                double robotLeft =
                        rampedLeft
                                * maxLinearVelocity;


                double cos =
                        Math.cos(targetHeading);

                double sin =
                        Math.sin(targetHeading);


                targetVx =
                        robotForward * cos
                                - robotLeft * sin;

                targetVy =
                        robotForward * sin
                                + robotLeft * cos;
            }


            targetOmega =
                    rampedTurn
                            * maxAngularVelocity;


            // =====================================================
            // MOVE THE TARGET
            //
            // Joystick controls target velocity.
            //
            // Road Runner controls actual robot X/Y/heading.
            // =====================================================

            targetX +=
                    0.5
                            * (oldVx + targetVx)
                            * dt;

            targetY +=
                    0.5
                            * (oldVy + targetVy)
                            * dt;

            targetHeading +=
                    0.5
                            * (oldOmega + targetOmega)
                            * dt;


            // =====================================================
            // TARGET ACCELERATION
            // =====================================================

            double targetAx =
                    (targetVx - oldVx) / dt;

            double targetAy =
                    (targetVy - oldVy) / dt;

            double targetAlpha =
                    (targetOmega - oldOmega) / dt;


            // =====================================================
            // COMPLETE ROAD RUNNER TARGET
            //
            // Each DualNum is:
            // [position, velocity, acceleration]
            // =====================================================

            Pose2dDual<Time> targetPose =
                    new Pose2dDual<>(
                            new DualNum<Time>(
                                    new double[]{
                                            targetX,
                                            targetVx,
                                            targetAx
                                    }
                            ),

                            new DualNum<Time>(
                                    new double[]{
                                            targetY,
                                            targetVy,
                                            targetAy
                                    }
                            ),

                            new DualNum<Time>(
                                    new double[]{
                                            targetHeading,
                                            targetOmega,
                                            targetAlpha
                                    }
                            )
                    );


            // =====================================================
            // MECANUM DRIVE HANDLES EVERYTHING FROM HERE
            //
            // X position
            // Y position
            // heading
            // velocity feedback
            // sleep / wake
            // motor commands
            // =====================================================

            drive.driveToTarget(
                    targetPose,
                    actualVelocity
            );


            // =====================================================
            // TELEMETRY
            // =====================================================

            actualPose =
                    drive.localizer.getPose();


            double positionError =
                    Math.hypot(
                            targetX - actualPose.position.x,
                            targetY - actualPose.position.y
                    );


            double headingError =
                    wrapAngle(
                            targetHeading
                                    - actualPose.heading.toDouble()
                    );


            double linearSpeed =
                    Math.hypot(
                            actualVelocity.linearVel.x,
                            actualVelocity.linearVel.y
                    );


            telemetry.addData(
                    "Drive Mode",
                    fieldCentric
                            ? "Field-Centric"
                            : "Robot-Centric"
            );

            telemetry.addData(
                    "Slow Mode",
                    slowMode
            );


            telemetry.addData(
                    "Pose",
                    "X %.2f   Y %.2f",
                    actualPose.position.x,
                    actualPose.position.y
            );

            telemetry.addData(
                    "Heading",
                    "%.2f deg",
                    Math.toDegrees(
                            actualPose.heading.toDouble()
                    )
            );


            telemetry.addData(
                    "Target",
                    "X %.2f   Y %.2f",
                    targetX,
                    targetY
            );

            telemetry.addData(
                    "Target Heading",
                    "%.2f deg",
                    Math.toDegrees(targetHeading)
            );


            telemetry.addData(
                    "Position Error",
                    "%.3f in",
                    positionError
            );

            telemetry.addData(
                    "Heading Error",
                    "%.3f deg",
                    Math.toDegrees(
                            headingError
                    )
            );


            telemetry.addData(
                    "Speed",
                    "%.3f in/s",
                    linearSpeed
            );

            telemetry.addData(
                    "Yaw Rate",
                    "%.3f deg/s",
                    Math.toDegrees(
                            actualVelocity.angVel
                    )
            );


            telemetry.addData(
                    "Pose Controller",
                    drive.isPoseControllerSleeping()
                            ? "SLEEP"
                            : "ACTIVE"
            );


            telemetry.addData(
                    "Ramped Forward",
                    "%.3f",
                    rampedForward
            );

            telemetry.addData(
                    "Ramped Left",
                    "%.3f",
                    rampedLeft
            );

            telemetry.addData(
                    "Ramped Turn",
                    "%.3f",
                    rampedTurn
            );


            telemetry.update();
        }


        stopDrive(drive);
    }


    private static double deadband(double value) {

        if (Math.abs(value) < DEADBAND) {
            return 0.0;
        }

        return value;
    }


    /*
     * Hard slew-rate limiter.
     *
     * The value can move toward target by at most
     * maximumChange this iteration.
     */
    private static double moveToward(
            double current,
            double target,
            double maximumChange) {

        double difference =
                target - current;


        if (difference > maximumChange) {
            return current + maximumChange;
        }

        if (difference < -maximumChange) {
            return current - maximumChange;
        }

        return target;
    }


    private static double wrapAngle(double angle) {

        while (angle > Math.PI) {
            angle -= 2.0 * Math.PI;
        }

        while (angle < -Math.PI) {
            angle += 2.0 * Math.PI;
        }

        return angle;
    }


    private static void stopDrive(MecanumDrive drive) {

        drive.leftFront.setPower(0);
        drive.leftBack.setPower(0);
        drive.rightBack.setPower(0);
        drive.rightFront.setPower(0);
    }
}