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

        Pose2d START_POSE = new Pose2d(-37, -64, Math.toRadians(90));

        double rightTruss1 = -59.5;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.toRadians(180), Math.toRadians(180), 15.55)
                .followTrajectorySequence(drive -> drive.trajectorySequenceBuilder(START_POSE)

                                .splineTo(new Vector2d(-33.5, -37), Math.toRadians(-320))

                .lineToLinearHeading(new Pose2d(-60, -38.5, Math.toRadians(-180)))

                .lineToConstantHeading(new Vector2d(-50, rightTruss1))
                .splineToConstantHeading(new Vector2d(-38, rightTruss1), Math.toRadians(0))
                .lineToConstantHeading(new Vector2d(20, rightTruss1))
                .splineToConstantHeading(new Vector2d(midX(20, 50), midY(rightTruss1, -43)), lineDirection(20, rightTruss1,50,-43))
                .lineToConstantHeading(new Vector2d(50, -43))

                .lineToConstantHeading(new Vector2d(42, -64))


                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}