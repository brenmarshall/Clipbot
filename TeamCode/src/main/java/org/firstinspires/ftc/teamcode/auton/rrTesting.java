package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RR Testing")
public class rrTesting extends LinearOpMode {

    public static double low = 1.0 * 30.71283;
    public static double medium = 11.0 * 30.71283;
    public static double high = 21.0 * 30.71283;
    public static double stack1 = 4 * 30.71283;
    public static double stack2 = 3 * 30.71283;
    public static double stack3 = 2 * 30.71283;
    public static double stack4 = 1 * 30.71283;
    public static double stack5 = 0 * 30.71283;
    public static double stackLift = 5 * 30.71283;
    public static double stackTime1 = 0.25;
    public static double stackTime2 = 0.25;
    public static double stackTime3 = 0.25;
    public static double scoreTime1 = 0.25;
    public static double scoreTime2 = 0.25;
    public static double open = 0.8;
    public static double closed = 1.0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() {
        DcMotor liftMotorLeft = hardwareMap.dcMotor.get("liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.dcMotor.get("liftMotorRight");

        liftMotorLeft.setTargetPosition(0);
        liftMotorRight.setTargetPosition(0);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        Servo gripServo = hardwareMap.servo.get("manipulator");

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90)); // Set start pose to center of the field, facing north
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score preload on high
                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) stack1);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) stack1);
                    liftMotorRight.setPower(1.0);
                })

                // Go to pick 1st cone off stack
                .setReversed(true)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                .waitSeconds(stackTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(closed);
                })
                .waitSeconds(stackTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) (stack1 + stackLift));
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) (stack1 + stackLift));
                    liftMotorRight.setPower(1.0);
                })
                .waitSeconds(stackTime3)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score 1st cone off stack on high
                .setReversed(false)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) stack2);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) stack2);
                    liftMotorRight.setPower(1.0);
                })

                // Go to pick 2nd cone off stack
                .setReversed(true)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                .waitSeconds(stackTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(closed);
                })
                .waitSeconds(stackTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) (stack2 + stackLift));
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) (stack2 + stackLift));
                    liftMotorRight.setPower(1.0);
                })
                .waitSeconds(stackTime3)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score 2nd cone off stack on low
                .setReversed(false)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) stack3);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) stack3);
                    liftMotorRight.setPower(1.0);
                })

                // Go to pick 3rd cone off stack
                .setReversed(true)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                .waitSeconds(stackTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(closed);
                })
                .waitSeconds(stackTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) (stack3 + stackLift));
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) (stack3 + stackLift));
                    liftMotorRight.setPower(1.0);
                })
                .waitSeconds(stackTime3)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score 3rd cone off stack on high
                .setReversed(false)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) stack4);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) stack4);
                    liftMotorRight.setPower(1.0);
                })

                // Go to pick 4th cone off stack
                .setReversed(true)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                .waitSeconds(stackTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(closed);
                })
                .waitSeconds(stackTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) (stack4 + stackLift));
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) (stack4 + stackLift));
                    liftMotorRight.setPower(1.0);
                })
                .waitSeconds(stackTime3)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score 4th cone off stack on medium
                .setReversed(false)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) stack5);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) stack5);
                    liftMotorRight.setPower(1.0);
                })

                // Go to pick 5th cone off stack
                .setReversed(true)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(180))
                .splineTo(new Vector2d(-62, -12), Math.toRadians(180))
                .waitSeconds(stackTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(closed);
                })
                .waitSeconds(stackTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition((int) high);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) high);
                    liftMotorRight.setPower(1.0);
                })

                // Go to score 5th cone off stack on high
                .setReversed(false)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineTo(new Vector2d(-30, -6), Math.toRadians(45))
                .waitSeconds(scoreTime1)
                .addTemporalMarker(() -> {
                    gripServo.setPosition(open);
                })
                .waitSeconds(scoreTime2)
                .addTemporalMarker(() -> {
                    liftMotorLeft.setTargetPosition(0);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition(0);
                    liftMotorRight.setPower(1.0);
                })

                // Go to park
                .setReversed(true)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))

                // Parking spot 2
                .splineTo(new Vector2d(-36, -36), Math.toRadians(270))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            dashboardTelemetry.update();
        }
    }
}