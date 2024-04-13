package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d START_POSE = new Pose2d(-37, 64, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.55)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POSE)

                        //ppp
                        .splineTo(new Vector2d(-34, 40), Math.toRadians(320))
                .lineToLinearHeading(new Pose2d(-58, 36, Math.toRadians(180)))
//                        //white stack
//                        .lineToLinearHeading(new Pose2d(-58, -36, Math.toRadians(180)))
//                        //slowly forward
//                        .forward(0.005)
//
//                        //to backdrop //true
//                        .turn(Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-34, -56), Math.toRadians(0))
//                        .lineToConstantHeading(new Vector2d(30, -56))
//                        .splineToConstantHeading(new Vector2d(46, -33), Math.toRadians(0))
//
//                        .waitSeconds(2)
//
//                        //to white stack
//                        .turn(Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(30, -56), Math.toRadians(180))
//                        .lineToConstantHeading(new Vector2d(-34, -56))
//                        .splineToConstantHeading(new Vector2d(-58, -36), Math.toRadians(180))
//
//                        .waitSeconds(2)
//
//                        //to backdrop //true
//                        .turn(Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(-34, -56), Math.toRadians(0))
//                        .lineToConstantHeading(new Vector2d(30, -56))
//                        .splineToConstantHeading(new Vector2d(46, -33), Math.toRadians(0))
//
//                        .waitSeconds(2)
//
//                        //park
//                        .turn(Math.toRadians(180))
//                        .splineToConstantHeading(new Vector2d(60, -56), Math.toRadians(330))

                        .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}