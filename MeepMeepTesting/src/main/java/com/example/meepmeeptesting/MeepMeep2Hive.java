package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import java.awt.Image;
import java.io.IOException;
import java.net.URL;
import java.util.Vector;

import javax.imageio.ImageIO;

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
public final class MeepMeep2Hive {

    /**
     * MeepMeep 0.1.7 does not yet include a built-in BIOBUZZ background enum.
     * This image was created by FTC Team Juice 16236 specifically as a
     * MeepMeep-compatible 2026-2027 BIOBUZZ field background and is loaded from
     * a pinned ftcontrol-panels commit so the image will not change unexpectedly.
     */
    private static final String BIOBUZZ_FIELD_URL =
            "https://raw.githubusercontent.com/ftcontrol/ftcontrol-panels/"
                    + "376eab5508adc3232dbe8f7cf520377b58ae2b35/"
                    + "library/Field/src/main/resources/biobuzz-dark.png";

    /**
     * Keep the constructor private because this class is only a desktop entry
     * point; we never create a MeepMeepTesting object.
     */
    private MeepMeep2Hive() {
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
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // maxVel, maxAccel, maxAngVel, maxAngAccel, trackWidth
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
        // Road Runner uses a 144 x 144 inch field centered at (0, 0), so the
        // field edges are approximately X/Y = +/-72 inches.
        // Heading 0 points along Road Runner's positive X direction.
        Pose2d startPose = new Pose2d(-62.0, 12, 0.0);

        // ------------------------------------------------------------------
        // STEP 4: BUILD AND RUN THE ROAD RUNNER PATH.
        // ------------------------------------------------------------------
        myBot.runAction(
                myBot.getDrive()
                        .actionBuilder(startPose)

                        .waitSeconds(4)
                        // Smooth curved path to (18, 18).
                        .strafeToLinearHeading(new Vector2d(-24,58), Math.toRadians(270))
                        .waitSeconds(4)
                        .strafeToLinearHeading(new Vector2d(48, 12), Math.toRadians(0))
                        // Move in a straight line until X = 30 inches.
                        .strafeToConstantHeading(new Vector2d(62,12))
                        .waitSeconds(4)
                        // Translate the mecanum robot directly to (30, 6).
                        .strafeToConstantHeading(new Vector2d(48, 24))
                        .strafeTo(new Vector2d(48,56))

                        // Mechanism-only events are represented by pauses because
                        // MeepMeep does not simulate the real lift or claw.

                        .build()
        );

        // ------------------------------------------------------------------
        // STEP 5: CONFIGURE THE 2026-2027 BIOBUZZ FIELD DISPLAY.
        // ------------------------------------------------------------------
        Image bioBuzzField = loadBioBuzzField();

        meepMeep
                .setBackground(bioBuzzField)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }

    /**
     * Downloads the pinned MeepMeep-compatible BIOBUZZ field background.
     *
     * Keeping this in one helper method makes it easy to replace with a future
     * built-in MeepMeep Background.FIELD_BIOBUZZ_* enum if the library adds one.
     */
    private static Image loadBioBuzzField() {
        try {
            URL fieldUrl = new URL(BIOBUZZ_FIELD_URL);
            Image fieldImage = ImageIO.read(fieldUrl);

            if (fieldImage == null) {
                throw new IllegalStateException(
                        "BIOBUZZ field image downloaded, but Java could not decode it."
                );
            }

            return fieldImage;
        } catch (IOException e) {
            throw new IllegalStateException(
                    "Unable to load the 2026-2027 BIOBUZZ MeepMeep field image. "
                            + "Check the computer's internet connection.",
                    e
            );
        }
    }
}
