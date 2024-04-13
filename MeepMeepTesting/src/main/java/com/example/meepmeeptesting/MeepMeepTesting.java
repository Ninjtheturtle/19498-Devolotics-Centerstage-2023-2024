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

        Pose2d START_POSE = new Pose2d(-37, 64, Math.toRadians(270));

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15.55)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POSE)

                                .lineToLinearHeading(new Pose2d(-33.5, 37, Math.toRadians(320)))

                                .lineToLinearHeading(new Pose2d(-58, 37, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(-50, 57))
                                .lineToConstantHeading(new Vector2d(30, 57))
                                .lineToConstantHeading(new Vector2d(48, 37))

                                .lineToConstantHeading(new Vector2d(30, 57))
                                .lineToConstantHeading(new Vector2d(-50, 57))

                                .lineToLinearHeading(new Pose2d(-58, 37, Math.toRadians(180)))

                                .lineToConstantHeading(new Vector2d(-50, 57))
                                .lineToConstantHeading(new Vector2d(30, 57))
                                .splineToConstantHeading(new Vector2d(midX(30,48), midY(57,37)), lineDirection(30,57,48,37))
                                .lineToConstantHeading(new Vector2d(48, 37))

                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}