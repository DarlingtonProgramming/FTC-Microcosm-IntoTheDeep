package com.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(700);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
            // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
            .setDimensions(16.75, 18.75)
            .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
            .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(36.5, 62, Math.toRadians(180)))
            .splineToLinearHeading(new Pose2d(10.5, 32, Math.toRadians(180)), Math.toRadians(270))
            .waitSeconds(0.01)
            .strafeToLinearHeading(new Vector2d(47, 47), Math.toRadians(270))
            .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(225))
            .strafeToLinearHeading(new Vector2d(58, 47), Math.toRadians(270))
            .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(225))
            .strafeToLinearHeading(new Vector2d(57, 40), Math.toRadians(315))
            .strafeToLinearHeading(new Vector2d(56, 56), Math.toRadians(225))
            .strafeToLinearHeading(new Vector2d(25, 12.5), Math.toRadians(180))
            .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
            .setDarkMode(true)
            .setBackgroundAlpha(0.95f)
            .addEntity(myBot)
            .start();
    }
}