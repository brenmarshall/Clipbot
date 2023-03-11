package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(52.48291908330528, 52.48291908330528, Math.toRadians(250.58748), Math.toRadians(250.58748), 12.5)
                .setDimensions(13, 13)
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(36, -63, Math.toRadians(90)))
                                // Go to score preload on low
                                .splineTo(new Vector2d(30, -54), Math.toRadians(135))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 1st cone off stack
                                .splineTo(new Vector2d(36, -36), Math.toRadians(90))
                                .splineTo(new Vector2d(36, -24), Math.toRadians(90))
                                .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 1st cone off stack on high
                                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(6, -18), Math.toRadians(225))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 2nd cone off stack
                                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 2nd cone off stack on low
                                .splineTo(new Vector2d(54, -18), Math.toRadians(225))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 3rd cone off stack
                                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 3rd cone off stack on high
                                .splineTo(new Vector2d(24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(6, -18), Math.toRadians(225))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 4th cone off stack
                                .splineTo(new Vector2d(24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 4th cone off stack on medium
                                .splineTo(new Vector2d(48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(30, -18), Math.toRadians(225))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 5th cone off stack
                                .splineTo(new Vector2d(48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(62, -12), Math.toRadians(0))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 5th cone off stack on high
                                .splineTo(new Vector2d(48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(30, -6), Math.toRadians(135))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to park
                                .splineTo(new Vector2d(36, -24), Math.toRadians(270))
                                // Parking spot 1
                                //.splineTo(new Vector2d(24, -36), Math.toRadians(180))
                                //.splineTo(new Vector2d(12, -36), Math.toRadians(270))
                                // Parking spot 2
                                //.splineTo(new Vector2d(36, -36), Math.toRadians(270))
                                // Parking spot 3
                                .splineTo(new Vector2d(48, -36), Math.toRadians(0))
                                .splineTo(new Vector2d(60, -36), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}