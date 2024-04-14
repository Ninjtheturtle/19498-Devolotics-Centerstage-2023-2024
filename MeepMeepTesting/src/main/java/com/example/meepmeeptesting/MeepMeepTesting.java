package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static double lineDirection(double startPointX, double startPointY, double endPointX, double endPointY){
        return Math.atan2(endPointY - startPointY, endPointX - startPointX);
    }
    public static double midX(double startPointX, double endPointX){
        return (endPointX + startPointX) / 2;
    }
    public static double midY(double startPointY, double endPointY){
        return (endPointY + startPointY) / 2;
    }
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        Pose2d START_POSE = new Pose2d(12, -64, Math.toRadians(-270));


        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.55)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POSE)

                                .lineToLinearHeading(new Pose2d(26, -28, Math.toRadians(180)))
                                .lineToConstantHeading(new Vector2d(50.3, -42))
                                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(midX(50.9, 40), midY(-41.4, -11), Math.toRadians(180))) //away from backdrop
                .splineToConstantHeading(new Vector2d(30, -11), Math.toRadians(180)) //connector
                .lineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180))) //straight to stack
                                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180))) //straight to backdrop
                .splineToSplineHeading(new Pose2d(53, -26, Math.toRadians(-210)), lineDirection(30, -11, 53, -26))//connector
                                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(midX(53, 30), midY(-26, -11), Math.toRadians(-210))) //away from backdrop
                .splineToSplineHeading(new Pose2d(30, -11, Math.toRadians(180)), Math.toRadians(180)) //connector
                .lineToLinearHeading(new Pose2d(-57, -11, Math.toRadians(180))) //straight to stack
                                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(30, -11, Math.toRadians(180))) //straight to backdrop
                .splineToSplineHeading(new Pose2d(53, -26, Math.toRadians(-210)), lineDirection(30, -11, 53, -26))//connector
                                .waitSeconds(1)

                .lineToLinearHeading(new Pose2d(48, -12, Math.toRadians(180)))

                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}