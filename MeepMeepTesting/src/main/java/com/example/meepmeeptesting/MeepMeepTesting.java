package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

/**
 * MEEPMEEP VISUALIZATION OF SampleMiddleFieldAuto
 *
 * This desktop program lets the team preview the Road Runner portion of the
 * autonomous without connecting to the physical FTC robot.
 *
 * IMPORTANT DIFFERENCE FROM THE REAL ROBOT:
 * MeepMeep simulates the robot's field movement, but it does not know about the
 * real Control Hub, leftLift motor, encoder, or claw servo. Because of that, the
 * drivetrain path can be visualized directly while the mechanism portion is
 * represented by comments and a short pause at the scoring location.
 *
 * The matching real autonomous is:
 * TeamCode/.../autonomous/SampleMiddleFieldAuto.java
 */
public final class MeepMeepTesting {

    /**
     * Keep the constructor private because this class is only a desktop entry
     * point; we never create a MeepMeepTesting object.
     */
    private MeepMeepTesting() {
    }

    public static void main(String[] args) {
        // ------------------------------------------------------------------
        // STEP 1: CREATE THE MEEPMEEP WINDOW.
        // ------------------------------------------------------------------
        // 800 is the simulator window size in pixels.
        MeepMeep meepMeep = new MeepMeep(800);

        // ------------------------------------------------------------------
        // STEP 2: CREATE THE SIMULATED FTC ROBOT.
        // ------------------------------------------------------------------
        // These lines were already required by this project's MeepMeep setup.
        // They define the virtual robot's motion limits and physical footprint.
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
                // These preserve the conservative values already used in this
                // project's original MeepMeepTesting.java.
                .setConstraints(
                        30.0,
                        20.0,
                        Math.PI,
                        Math.PI,
                        13.810911318867477
                )

                // Approximate 18 x 18 inch FTC robot footprint displayed on field.
                .setDimensions(18.0, 18.0)
                .build();

        // ------------------------------------------------------------------
        // STEP 3: DEFINE THE SAME STARTING POSE AS THE REAL AUTONOMOUS.
        // ------------------------------------------------------------------
        // X = 0 inches, Y = 0 inches, heading = 0 radians.
        // Heading 0 points along Road Runner's positive X direction.
        Pose2d startPose = new Pose2d(0.0, 0.0, 0.0);

        // ------------------------------------------------------------------
        // STEP 4: BUILD AND RUN THE ROAD RUNNER PATH.
        // ------------------------------------------------------------------
        // myBot.getDrive().actionBuilder(...) is the MeepMeep equivalent of the
        // drive.roadRunner().actionBuilder(...) call used on the real robot.
        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)

                        // --------------------------------------------------
                        // SPLINE MOVEMENT
                        // --------------------------------------------------
                        // The robot leaves the center of the field and follows a
                        // smooth curved path to (18, 18). The 45-degree tangent
                        // tells Road Runner the direction of the path at the end.
                        .splineTo(
                                new Vector2d(18.0, 18.0),
                                Math.toRadians(45.0)
                        )

                        // --------------------------------------------------
                        // LINE MOVEMENT
                        // --------------------------------------------------
                        // Move in a straight line until X = 30 inches. This is the
                        // same lineToX() command used in SampleMiddleFieldAuto.
                        .lineToX(30.0)

                        // --------------------------------------------------
                        // STRAFE MOVEMENT
                        // --------------------------------------------------
                        // Translate the mecanum robot directly to (30, 6).
                        // This demonstrates sideways/diagonal holonomic movement.
                        .strafeTo(new Vector2d(30.0, 6.0))

                        // --------------------------------------------------
                        // MECHANISM EVENT REPRESENTATION
                        // --------------------------------------------------
                        // On the REAL robot, this is where the following happens:
                        //
                        // new ParallelAction(
                        //     raiseLiftHigh,
                        //     openClaw
                        // )
                        //
                        // MeepMeep cannot animate our physical leftLift motor or
                        // claw servo. The pause gives students a visible indication
                        // that the robot has reached the mechanism/scoring step.
                        .waitSeconds(1.0)

                        // On the REAL robot, the SequentialAction then continues
                        // with lowerLiftHome after the lift-high/claw-open parallel
                        // step has completed. There is no drivetrain motion during
                        // that mechanism-only step, so the simulated robot remains
                        // at (30, 6).
                        .waitSeconds(1.0)

                        // Convert the trajectory description into a Road Runner
                        // Action that MeepMeep can execute and animate.
                        .build()
        );

        // ------------------------------------------------------------------
        // STEP 5: CONFIGURE THE MEEPMEEP FIELD DISPLAY.
        // ------------------------------------------------------------------
        // These lines are required to display the field and simulated robot.
        // POWERPLAY remains the background already configured in this project;
        // the trajectory itself uses our generic center-of-field coordinates.
        meepMeep
                .setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
