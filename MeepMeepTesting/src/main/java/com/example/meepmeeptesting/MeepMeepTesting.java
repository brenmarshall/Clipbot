package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double stackTime1 = 0.25;
        double stackTime2 = 0.25;
        double stackTime3 = 0.25;
        double scoreTime1 = 0.25;
        double scoreTime2 = 0.25;
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(250.58748), Math.toRadians(250.58748), 12.5)
                .setDimensions(13, 13)
                .setDriveTrainType(DriveTrainType.MECANUM)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                                .addTemporalMarker(() -> {
                                    //targetInches = low;
                                })

                                // Go to score preload on low
                                .splineTo(new Vector2d(-30, -54), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack1;
                                })

                                // Go to pick 1st cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })

                                // Go to score 1st cone off stack on high
                                .setReversed(false)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack2;
                                })

                                // Go to pick 2nd cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //targetInches = low;
                                })

                                // Go to score 2nd cone off stack on low
                                .setReversed(false)
                                .splineTo(new Vector2d(-54, -18), Math.toRadians(305))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack3;
                                })

                                // Go to pick 3rd cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })

                                // Go to score 3rd cone off stack on high
                                .setReversed(false)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack4;
                                })

                                // Go to pick 4th cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //targetInches = medium;
                                })

                                // Go to score 4th cone off stack on medium
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack5;
                                })

                                // Go to pick 5th cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })

                                // Go to score 5th cone off stack on high
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //targetInches = 0;
                                })

                                // Go to park
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))

                                // Parking spot 3
                                .splineToConstantHeading(new Vector2d(-24, -36), Math.toRadians(0))
                                .splineToConstantHeading(new Vector2d(-12, -36), Math.toRadians(0))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}