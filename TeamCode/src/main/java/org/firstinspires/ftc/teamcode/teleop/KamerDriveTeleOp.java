package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.acmerobotics.roadrunner.DualNum;
import com.acmerobotics.roadrunner.HolonomicController;
import com.acmerobotics.roadrunner.MecanumKinematics;
import com.acmerobotics.roadrunner.MotorFeedforward;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Pose2dDual;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.PoseVelocity2dDual;
import com.acmerobotics.roadrunner.Time;
import com.acmerobotics.roadrunner.Vector2d;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.drive.MecanumDrive;


@Config
@TeleOp(name = "Kamer Drive TeleOp", group = "TeleOp")
public final class KamerDriveTeleOp extends LinearOpMode {

    /*
     * =========================================================
     * GAMEPAD 1
     * =========================================================
     *
     * Left stick:
     *     translation
     *
     * Right stick X:
     *     yaw
     *
     * A:
     *     toggle FAST / SLOW
     *
     * Y:
     *     toggle ROBOT-CENTRIC / FIELD-CENTRIC
     *
     * OPTIONS / START:
     *     reset heading to 0
     *
     *
     * =========================================================
     * STEP 1 OUTPUTS
     * =========================================================
     *
     * targetVxRobot
     *     requested robot-forward velocity, inches/sec
     *
     * targetVyRobot
     *     requested robot-left velocity, inches/sec
     *
     * targetYawRate
     *     requested yaw velocity, radians/sec
     *
     * angleA
     *     angle of requested translation relative to robot heading
     *
     *     0 = robot-forward
     *     +pi/2 = robot-left
     *
     *     If requested translation is zero:
     *         angleA = 0
     */


    // =========================================================
    // TOP SPEEDS
    // =========================================================

    public static double FAST_TRANSLATION_SPEED = 80.0;
    public static double SLOW_TRANSLATION_SPEED = 32.0;

    public static double FAST_YAW_SPEED = Math.PI;
    public static double SLOW_YAW_SPEED = 0.40 * Math.PI;


    // =========================================================
    // RAMP SPEEDS
    //
    // These are ABSOLUTE ramp rates.
    //
    // Slow and fast mode use exactly the same ramp.
    // Therefore slow mode reaches its lower maximum speed sooner.
    // =========================================================

    public static double TRANSLATION_RAMP = 320.0;   // in/s^2
    public static double YAW_RAMP = 4.0 * Math.PI;   // rad/s^2


    private static final double STICK_DEADBAND = 0.05;


    // =========================================================
    // TELEOP BRAKE / SETTLED THRESHOLDS
    //
    // These belong to this TeleOp. MecanumDrive does not need
    // any special sleep/wake modification for this controller.
    // =========================================================

    public static double BRAKE_POSITION_ERROR = 0.15;
    public static double BRAKE_HEADING_ERROR = Math.toRadians(0.75);
    public static double BRAKE_LINEAR_SPEED = 0.50;
    public static double BRAKE_YAW_RATE = Math.toRadians(2.0);


    @Override
    public void runOpMode() {

        // =========================================================
        // ROAD RUNNER / LOCALIZATION
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
        boolean previousHeadingReset = false;


        // =========================================================
        // RAMP STATE
        //
        // In robot-centric mode this vector is robot-relative.
        //
        // In field-centric mode this vector is field-relative.
        // It is converted to robot-relative after ramping.
        // =========================================================

        Vector2d rampedTranslation =
                new Vector2d(0.0, 0.0);

        double rampedYawRate = 0.0;


        // =========================================================
        // STEP 2 STATE
        // =========================================================

        double previousAngleA = 0.0;


        // =========================================================
        // INITIALIZE LOCALIZATION
        // =========================================================

        drive.updatePoseEstimate();

        Pose2d initialPose =
                drive.localizer.getPose();


        // =========================================================
        // STEP 3 STATE
        // =========================================================

        double targetX =
                initialPose.position.x;

        double targetY =
                initialPose.position.y;

        double targetHeading =
                initialPose.heading.toDouble();

        double previousTargetVxField = 0.0;
        double previousTargetVyField = 0.0;
        double previousTargetYawRate = 0.0;


        telemetry.addLine("Drive initialized");
        telemetry.addData("Speed Mode (A)", "FAST");
        telemetry.addData("Drive Mode (Y)", "ROBOT-CENTRIC");
        telemetry.addData(
                "Heading (OPTIONS/START = reset)",
                "%.1f deg",
                0.0
        );
        telemetry.update();


        waitForStart();


        if (isStopRequested()) {
            stopDrive(drive);
            return;
        }


        long previousTime = System.nanoTime();


        while (opModeIsActive()) {

            // =====================================================
            // LOOP TIME
            // =====================================================

            long now = System.nanoTime();

            double dt =
                    (now - previousTime) * 1e-9;

            previousTime = now;


            if (dt <= 0.0 || dt > 0.1) {
                dt = 0.02;
            }


            // =====================================================
            // ACTUAL ROBOT HEADING
            // =====================================================

            PoseVelocity2d actualVelocity =
                    drive.updatePoseEstimate();

            Pose2d actualPose =
                    drive.localizer.getPose();

            double heading =
                    actualPose.heading.toDouble();


            // =====================================================
            // BUTTONS
            // =====================================================

            boolean aPressed =
                    gamepad1.a;

            boolean yPressed =
                    gamepad1.y;

            boolean headingResetPressed =
                    gamepad1.options
                            || gamepad1.start;


            // =====================================================
            // FAST / SLOW
            // =====================================================

            if (aPressed && !previousA) {
                slowMode = !slowMode;
            }


            // =====================================================
            // FIELD / ROBOT CENTRIC
            // =====================================================

            if (yPressed && !previousY) {

                double cos =
                        Math.cos(heading);

                double sin =
                        Math.sin(heading);


                if (fieldCentric) {

                    /*
                     * We are switching:
                     *
                     * FIELD -> ROBOT
                     *
                     * Convert the current ramp state so the requested
                     * velocity does not jump simply because the mode
                     * was toggled.
                     */

                    rampedTranslation =
                            new Vector2d(
                                    rampedTranslation.x * cos
                                            + rampedTranslation.y * sin,

                                    -rampedTranslation.x * sin
                                            + rampedTranslation.y * cos
                            );

                } else {

                    /*
                     * We are switching:
                     *
                     * ROBOT -> FIELD
                     */

                    rampedTranslation =
                            new Vector2d(
                                    rampedTranslation.x * cos
                                            - rampedTranslation.y * sin,

                                    rampedTranslation.x * sin
                                            + rampedTranslation.y * cos
                            );
                }


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

                heading =
                        actualPose.heading.toDouble();

                // Keep the heading target synchronized with the reset.
                targetHeading = heading;
            }


            previousA = aPressed;
            previousY = yPressed;
            previousHeadingReset = headingResetPressed;


            // =====================================================
            // RAW CONTROLLER INPUT
            //
            // Positive X = forward
            // Positive Y = left
            // Positive yaw = counterclockwise
            // =====================================================

            double stickX =
                    -gamepad1.left_stick_y;

            double stickY =
                    -gamepad1.left_stick_x;

            double yawInput =
                    -gamepad1.right_stick_x;


            // =====================================================
            // LEFT-STICK DEADBAND
            // =====================================================

            double stickMagnitude =
                    Math.hypot(stickX, stickY);


            if (stickMagnitude < STICK_DEADBAND) {

                stickX = 0.0;
                stickY = 0.0;

            } else if (stickMagnitude > 1.0) {

                stickX /= stickMagnitude;
                stickY /= stickMagnitude;
            }


            if (Math.abs(yawInput) < STICK_DEADBAND) {
                yawInput = 0.0;
            }


            // =====================================================
            // SELECT TOP SPEED
            // =====================================================

            double maximumTranslationSpeed =
                    slowMode
                            ? SLOW_TRANSLATION_SPEED
                            : FAST_TRANSLATION_SPEED;


            double maximumYawSpeed =
                    slowMode
                            ? SLOW_YAW_SPEED
                            : FAST_YAW_SPEED;


            // =====================================================
            // DESIRED VELOCITIES BEFORE RAMP
            // =====================================================

            Vector2d desiredTranslation =
                    new Vector2d(
                            stickX * maximumTranslationSpeed,
                            stickY * maximumTranslationSpeed
                    );


            double desiredYawRate =
                    yawInput * maximumYawSpeed;


            // =====================================================
            // TRANSLATION RAMP
            //
            // One 2D ramp.
            //
            // This limits the magnitude of the CHANGE in the whole
            // translation vector rather than independently ramping
            // X and Y.
            // =====================================================

            double maximumTranslationChange =
                    TRANSLATION_RAMP * dt;


            rampedTranslation =
                    moveVectorToward(
                            rampedTranslation,
                            desiredTranslation,
                            maximumTranslationChange
                    );


            // =====================================================
            // YAW RAMP
            // =====================================================

            double maximumYawChange =
                    YAW_RAMP * dt;


            rampedYawRate =
                    moveToward(
                            rampedYawRate,
                            desiredYawRate,
                            maximumYawChange
                    );


            // =====================================================
            // STEP 1 OUTPUT:
            // ROBOT-CENTRIC TRANSLATION VELOCITY
            // =====================================================

            double targetVxRobot;
            double targetVyRobot;


            if (fieldCentric) {

                /*
                 * rampedTranslation is currently FIELD-relative.
                 *
                 * Rotate by -heading to express the request
                 * relative to the robot.
                 */

                double cos =
                        Math.cos(heading);

                double sin =
                        Math.sin(heading);


                targetVxRobot =
                        rampedTranslation.x * cos
                                + rampedTranslation.y * sin;


                targetVyRobot =
                        -rampedTranslation.x * sin
                                + rampedTranslation.y * cos;

            } else {

                /*
                 * Already robot-relative.
                 */

                targetVxRobot =
                        rampedTranslation.x;

                targetVyRobot =
                        rampedTranslation.y;
            }


            // =====================================================
            // STEP 1 OUTPUT:
            // YAW VELOCITY
            // =====================================================

            double targetYawRate =
                    rampedYawRate;


            // =====================================================
            // STEP 1 OUTPUT:
            // angleA
            //
            // Angle of the ACTUAL RAMPED TRANSLATION REQUEST
            // relative to the robot.
            // =====================================================

            double requestedTranslationSpeed =
                    Math.hypot(
                            targetVxRobot,
                            targetVyRobot
                    );


            double angleA;


            if (requestedTranslationSpeed == 0.0) {

                angleA = 0.0;

            } else {

                angleA =
                        Math.atan2(
                                targetVyRobot,
                                targetVxRobot
                        );
            }


            // =====================================================
            // STEP 2:
            // CHANGE IN TRANSLATION ANGLE
            // =====================================================

            double angleDifference =
                    angleA - previousAngleA;


            /*
             * angleA comes from atan2(), so it is in [-pi, pi].
             *
             * Correct only the numerical wraparound. For example,
             * +179 deg -> -179 deg is a 2 deg change, not 358 deg.
             */
            while (angleDifference > Math.PI) {
                angleDifference -= 2.0 * Math.PI;
            }

            while (angleDifference < -Math.PI) {
                angleDifference += 2.0 * Math.PI;
            }


            double aPrime =
                    angleDifference / dt;


            // =====================================================
            // STEP 3A:
            // CURRENT ROAD RUNNER POSITION ERROR
            //
            // targetX/targetY are stored in FIELD coordinates.
            // Convert their error into ROBOT coordinates before
            // applying the Stage 2 equations.
            // =====================================================

            double xErrorField =
                    targetX - actualPose.position.x;

            double yErrorField =
                    targetY - actualPose.position.y;


            double cosHeading =
                    Math.cos(heading);

            double sinHeading =
                    Math.sin(heading);


            double xErrorRobot =
                    xErrorField * cosHeading
                            + yErrorField * sinHeading;

            double yErrorRobot =
                    -xErrorField * sinHeading
                            + yErrorField * cosHeading;


            // =====================================================
            // STEP 2 OUTPUT:
            // aOne = (x1, y1)
            //
            // These are exactly the two equations from Stage 2.
            // They operate on the robot-relative position errors.
            // =====================================================

            Vector2d aOne;


            if (requestedTranslationSpeed > 0.0) {

                aOne =
                        calculateAOne(
                                xErrorRobot,
                                yErrorRobot,
                                previousAngleA,
                                angleA,
                                aPrime,
                                dt
                        );


                // =================================================
                // STEP 3B:
                // ROBOT POSITION CORRECTION -> FIELD TARGET
                // =================================================

                double xOneField =
                        aOne.x * cosHeading
                                - aOne.y * sinHeading;

                double yOneField =
                        aOne.x * sinHeading
                                + aOne.y * cosHeading;


                targetX =
                        actualPose.position.x
                                + xOneField;

                targetY =
                        actualPose.position.y
                                + yOneField;

            } else {

                /*
                 * angleA = 0 is only a placeholder when translation
                 * is zero; it is not a real direction.
                 *
                 * Do not run the angle equations in that state.
                 * Leave targetX/targetY fixed so Road Runner can
                 * finish correcting to the last target, then sleep.
                 */
                aOne =
                        new Vector2d(
                                xErrorRobot,
                                yErrorRobot
                        );
            }


            // Save a(t) for the next loop's a(t-1).
            previousAngleA = angleA;


            // =====================================================
            // STEP 3C:
            // ROBOT VELOCITY REQUEST -> FIELD VELOCITY REQUEST
            // =====================================================

            double targetVxField =
                    targetVxRobot * cosHeading
                            - targetVyRobot * sinHeading;

            double targetVyField =
                    targetVxRobot * sinHeading
                            + targetVyRobot * cosHeading;


            // =====================================================
            // STEP 3D:
            // TARGET ACCELERATION FROM THE RAMPED VELOCITY CHANGE
            // =====================================================

            double targetAxField =
                    (targetVxField - previousTargetVxField)
                            / dt;

            double targetAyField =
                    (targetVyField - previousTargetVyField)
                            / dt;

            double targetAlpha =
                    (targetYawRate - previousTargetYawRate)
                            / dt;


            // =====================================================
            // STEP 3E:
            // HEADING TARGET
            //
            // While a ramped yaw request exists, heading follows the
            // actual robot heading so Road Runner does not fight the
            // driver's requested yaw velocity.
            //
            // On the first loop after the yaw ramp reaches zero,
            // capture the actual heading. From then on it is held.
            // =====================================================

            boolean yawRequested =
                    Math.abs(targetYawRate) > 1e-9;

            boolean yawWasRequested =
                    Math.abs(previousTargetYawRate) > 1e-9;


            if (yawRequested || yawWasRequested) {
                targetHeading = heading;
            }


            // =====================================================
            // STEP 3F:
            // COMPLETE ROAD RUNNER TARGET
            //
            // Each DualNum is:
            // [position, velocity, acceleration]
            // =====================================================

            Pose2dDual<Time> roadRunnerTarget =
                    new Pose2dDual<>(
                            new DualNum<Time>(
                                    new double[]{
                                            targetX,
                                            targetVxField,
                                            targetAxField
                                    }
                            ),

                            new DualNum<Time>(
                                    new double[]{
                                            targetY,
                                            targetVyField,
                                            targetAyField
                                    }
                            ),

                            new DualNum<Time>(
                                    new double[]{
                                            targetHeading,
                                            targetYawRate,
                                            targetAlpha
                                    }
                            )
                    );


            // =====================================================
            // STEP 3G:
            // DIRECT HOLONOMIC CONTROLLER / BRAKE
            //
            // This is based on the POST-RAMP request.
            // Raw stick release does not immediately request sleep.
            // =====================================================

            boolean motionRequested =
                    requestedTranslationSpeed > 1e-9
                            || Math.abs(targetYawRate) > 1e-9;


            runHolonomicController(
                    drive,
                    roadRunnerTarget,
                    actualPose,
                    actualVelocity,
                    motionRequested
            );


            previousTargetVxField = targetVxField;
            previousTargetVyField = targetVyField;
            previousTargetYawRate = targetYawRate;


            // =====================================================
            // TELEMETRY
            // =====================================================

            telemetry.addData(
                    "Speed Mode (A)",
                    slowMode
                            ? "SLOW"
                            : "FAST"
            );


            telemetry.addData(
                    "Drive Mode (Y)",
                    fieldCentric
                            ? "FIELD-CENTRIC"
                            : "ROBOT-CENTRIC"
            );


            telemetry.addData(
                    "Heading (OPTIONS/START = reset)",
                    "%.1f deg",
                    Math.toDegrees(heading)
            );


            telemetry.update();
        }


        stopDrive(drive);
    }


    // =============================================================
    // STEP 3 - DIRECT ROAD RUNNER HOLONOMIC CONTROLLER
    //
    // This does NOT call a custom/modified MecanumDrive method.
    // It uses the public Road Runner hardware/model pieces already
    // exposed by MecanumDrive:
    //
    //     localizer pose/velocity
    //     HolonomicController
    //     mecanum kinematics
    //     motor feedforward
    //     drive motors
    //
    // If the post-ramp driver request is zero AND the robot is
    // already settled at the requested pose, motor power is set to
    // zero so the drivetrain's ZeroPowerBehavior.BRAKE can hold it.
    // =============================================================

    private static void runHolonomicController(
            MecanumDrive drive,
            Pose2dDual<Time> targetPose,
            Pose2d currentPose,
            PoseVelocity2d robotVelocity,
            boolean motionRequested) {

        Pose2d target =
                targetPose.value();


        // ---------------------------------------------------------
        // Check whether the robot is already fully settled.
        // ---------------------------------------------------------

        double xError =
                target.position.x - currentPose.position.x;

        double yError =
                target.position.y - currentPose.position.y;

        double positionError =
                Math.hypot(xError, yError);


        double headingError =
                wrapAngle(
                        target.heading.toDouble()
                                - currentPose.heading.toDouble()
                );


        double linearSpeed =
                Math.hypot(
                        robotVelocity.linearVel.x,
                        robotVelocity.linearVel.y
                );

        double yawRate =
                Math.abs(robotVelocity.angVel);


        boolean settled =
                positionError < BRAKE_POSITION_ERROR
                        && Math.abs(headingError) < BRAKE_HEADING_ERROR
                        && linearSpeed < BRAKE_LINEAR_SPEED
                        && yawRate < BRAKE_YAW_RATE;


        if (!motionRequested && settled) {
            stopDrive(drive);
            return;
        }


        // ---------------------------------------------------------
        // Road Runner HolonomicController
        // ---------------------------------------------------------

        PoseVelocity2dDual<Time> command =
                new HolonomicController(
                        MecanumDrive.PARAMS.axialGain,
                        MecanumDrive.PARAMS.lateralGain,
                        MecanumDrive.PARAMS.headingGain,
                        MecanumDrive.PARAMS.axialVelGain,
                        MecanumDrive.PARAMS.lateralVelGain,
                        MecanumDrive.PARAMS.headingVelGain
                ).compute(
                        targetPose,
                        currentPose,
                        robotVelocity
                );


        // ---------------------------------------------------------
        // Chassis command -> mecanum wheel velocities
        // ---------------------------------------------------------

        MecanumKinematics.WheelVelocities<Time> wheelVels =
                drive.kinematics.inverse(command);


        // ---------------------------------------------------------
        // Wheel velocities/accelerations -> motor power
        // ---------------------------------------------------------

        double voltage =
                drive.voltageSensor.getVoltage();


        MotorFeedforward feedforward =
                new MotorFeedforward(
                        MecanumDrive.PARAMS.kS,
                        MecanumDrive.PARAMS.kV
                                / MecanumDrive.PARAMS.inPerTick,
                        MecanumDrive.PARAMS.kA
                                / MecanumDrive.PARAMS.inPerTick
                );


        double leftFrontPower =
                feedforward.compute(wheelVels.leftFront)
                        / voltage;

        double leftBackPower =
                feedforward.compute(wheelVels.leftBack)
                        / voltage;

        double rightBackPower =
                feedforward.compute(wheelVels.rightBack)
                        / voltage;

        double rightFrontPower =
                feedforward.compute(wheelVels.rightFront)
                        / voltage;


        // Keep all four wheel-power ratios unchanged if one exceeds 1.
        double maxPower =
                Math.max(
                        1.0,
                        Math.max(
                                Math.max(
                                        Math.abs(leftFrontPower),
                                        Math.abs(leftBackPower)
                                ),
                                Math.max(
                                        Math.abs(rightBackPower),
                                        Math.abs(rightFrontPower)
                                )
                        )
                );


        leftFrontPower /= maxPower;
        leftBackPower /= maxPower;
        rightBackPower /= maxPower;
        rightFrontPower /= maxPower;


        drive.leftFront.setPower(leftFrontPower);
        drive.leftBack.setPower(leftBackPower);
        drive.rightBack.setPower(rightBackPower);
        drive.rightFront.setPower(rightFrontPower);
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


    // =============================================================
    // STEP 2 - ANGLE-DEPENDENT POSITION CORRECTION
    // =============================================================

    private static Vector2d calculateAOne(
            double x,
            double y,
            double previousAngleA,
            double angleA,
            double aPrime,
            double dt) {

        double angleChangeFraction =
                (2.0 * aPrime / Math.PI) * dt;


        double xOne =
                Math.abs(Math.cos(previousAngleA))
                        * x
                        * (1.0 - angleChangeFraction)

                        +

                        Math.abs(Math.cos(angleA))
                                * x
                                * angleChangeFraction;


        double yOne =
                Math.abs(Math.sin(previousAngleA))
                        * y
                        * (1.0 - angleChangeFraction)

                        +

                        Math.abs(Math.sin(angleA))
                                * y
                                * angleChangeFraction;


        return new Vector2d(
                xOne,
                yOne
        );
    }


    // =============================================================
    // 2D TRANSLATION RAMP
    // =============================================================

    private static Vector2d moveVectorToward(
            Vector2d current,
            Vector2d target,
            double maximumChange) {

        double differenceX =
                target.x - current.x;

        double differenceY =
                target.y - current.y;


        double differenceMagnitude =
                Math.hypot(
                        differenceX,
                        differenceY
                );


        if (differenceMagnitude == 0.0
                || differenceMagnitude <= maximumChange) {

            return target;
        }


        double scale =
                maximumChange
                        / differenceMagnitude;


        return new Vector2d(
                current.x
                        + differenceX * scale,

                current.y
                        + differenceY * scale
        );
    }


    // =============================================================
    // 1D YAW RAMP
    // =============================================================

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


    private static void stopDrive(
            MecanumDrive drive) {

        drive.leftFront.setPower(0.0);
        drive.leftBack.setPower(0.0);
        drive.rightBack.setPower(0.0);
        drive.rightFront.setPower(0.0);
    }
}
