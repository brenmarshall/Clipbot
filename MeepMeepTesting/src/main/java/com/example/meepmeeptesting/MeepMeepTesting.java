package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.profile.VelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.DriveTrainType;
import com.noahbres.meepmeep.roadrunner.SampleTankDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(500);
        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(63, 63, Math.toRadians(160), Math.toRadians(160), 13.95)
                .setDimensions(13, 13)
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                                // Go to score preload on low
                                .splineTo(new Vector2d(-30, -54), Math.toRadians(45))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 1st cone off stack
                                .splineTo(new Vector2d(-36, -36), Math.toRadians(90))
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 1st cone off stack on high
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 2nd cone off stack
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 2nd cone off stack on low
                                .splineTo(new Vector2d(-54, -18), Math.toRadians(315))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 3rd cone off stack
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 3rd cone off stack on high
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-6, -18), Math.toRadians(315))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 4th cone off stack
                                .splineTo(new Vector2d(-24, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 4th cone off stack on medium
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -18), Math.toRadians(315))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to pick 5th cone off stack
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .setReversed(false)
                                .waitSeconds(0.5)
                                // Go to score 5th cone off stack on high
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(0.5)
                                .setReversed(true)
                                // Go to park
                                .splineTo(new Vector2d(-36, -12), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}