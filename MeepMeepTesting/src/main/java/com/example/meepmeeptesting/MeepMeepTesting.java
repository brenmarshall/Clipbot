package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        double stackTime = 0.25;
        double scoreTime = 0.25;
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(250.58748), Math.toRadians(250.58748), 12.5)
                .setDimensions(13, 13)
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                                // Go to score preload on low
                                .addTemporalMarker(() -> {
                                    //targetInches = low;
                                })
                                .splineTo(new Vector2d(-30, -54), Math.toRadians(45))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack1;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime)
                                // Go to score 1st cone off stack on high
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .setReversed(false)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack2;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime)
                                // Go to score 1st cone off stack on high
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = low;
                                })
                                .setReversed(false)
                                .splineTo(new Vector2d(-54, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack3;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime)
                                // Go to score 1st cone off stack on high
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .setReversed(false)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack4;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime)
                                // Go to score 1st cone off stack on high
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = medium;
                                })
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -18), Math.toRadians(315))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = stack5;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime)
                                // Go to score 1st cone off stack on high
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = targetInches + 5;
                                })
                                .waitSeconds(stackTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = high;
                                })
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime)
                                // Go to pick 1st cone off stack
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime)
                                .addTemporalMarker(() -> {
                                    //targetInches = 0;
                                })
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                                // Parking spot 1
                                .splineTo(new Vector2d(-48, -36), Math.toRadians(180))
                                .splineTo(new Vector2d(-60, -36), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}