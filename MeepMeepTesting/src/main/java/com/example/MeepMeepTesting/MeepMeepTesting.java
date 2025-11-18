package com.example.MeepMeepTesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(60, 60, Math.toRadians(180), Math.toRadians(180), 15)
                .build();


        // Basic auto: shoot 3 into depot; collect 3 from ground; shoot 3 into depot
        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(62, 12, (Math.toRadians(180))))
                .lineToX(55)
                .turn(Math.toRadians(-20))
                        .waitSeconds(6.7)
                .turn(Math.toRadians(20))
                .lineToX(35)
                .turn(Math.toRadians(-90))
                .lineToY(60)
                .lineToY(0)
                .turn(Math.toRadians(90))
                .lineToX(0)
                .turn(Math.toRadians(-45))
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}