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
                .setDriveTrainType(DriveTrainType.TANK)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-36, -63, Math.toRadians(90)))
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to score preload on high
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) stack1);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) stack1);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to pick 1st cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) (stack1 + stackLift));
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) (stack1 + stackLift));
                                    //liftMotorRight.setPower(1.0);
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to score 1st cone off stack on high
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) stack2);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) stack2);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to pick 2nd cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) (stack2 + stackLift));
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) (stack2 + stackLift));
                                    //liftMotorRight.setPower(1.0);
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to score 2nd cone off stack on low
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) stack3);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) stack3);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to pick 3rd cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) (stack3 + stackLift));
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) (stack3 + stackLift));
                                    //liftMotorRight.setPower(1.0);
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to score 3rd cone off stack on high
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) stack4);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) stack4);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to pick 4th cone off stack
                                .setReversed(true)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                                .waitSeconds(stackTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(closed);
                                })
                                .waitSeconds(stackTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) (stack4 + stackLift));
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) (stack4 + stackLift));
                                    //liftMotorRight.setPower(1.0);
                                })
                                .waitSeconds(stackTime3)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to score 4th cone off stack on medium
                                .setReversed(false)
                                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                                .waitSeconds(scoreTime1)
                                .addTemporalMarker(() -> {
                                    //gripServo.setPosition(open);
                                })
                                .waitSeconds(scoreTime2)
                                .addTemporalMarker(() -> {
                                    //liftMotorLeft.setTargetPosition((int) stack5);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) stack5);
                                    //liftMotorRight.setPower(1.0);
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
                                    //liftMotorLeft.setTargetPosition((int) high);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition((int) high);
                                    //liftMotorRight.setPower(1.0);
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
                                    //liftMotorLeft.setTargetPosition(0);
                                    //liftMotorLeft.setPower(1.0);
                                    //liftMotorRight.setTargetPosition(0);
                                    //liftMotorRight.setPower(1.0);
                                })

                                // Go to park
                                .setReversed(true)
                                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))

                                // Parking spot 2
                                .splineTo(new Vector2d(-36, -36), Math.toRadians(270))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}