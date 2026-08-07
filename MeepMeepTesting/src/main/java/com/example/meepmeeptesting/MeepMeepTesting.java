package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public final class MeepMeepTesting {
    private static final double SIDE_LENGTH_IN = 24.0;

    private MeepMeepTesting() {
    }

    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Mirrors SquareTestAuto's conservative test limits.
                .setConstraints(30.0, 20.0, Math.PI, Math.PI, 13.810911318867477)
                .setDimensions(18.0, 18.0)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(0.0, 0.0, 0.0))
                .strafeToConstantHeading(new Vector2d(SIDE_LENGTH_IN, 0.0))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(SIDE_LENGTH_IN, SIDE_LENGTH_IN))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(0.0, SIDE_LENGTH_IN))
                .waitSeconds(0.5)
                .strafeToConstantHeading(new Vector2d(0.0, 0.0))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}
