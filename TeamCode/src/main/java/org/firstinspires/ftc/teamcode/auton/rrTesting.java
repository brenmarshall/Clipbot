package org.firstinspires.ftc.teamcode.auton;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name="RR Testing")
public class rrTesting extends LinearOpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() {
        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90)); // Set start pose to center of the field, facing north
        drive.setPoseEstimate(startPose);

        TrajectorySequence trajSeq = drive.trajectorySequenceBuilder(startPose)
                .splineTo(new Vector2d(-36, -24), Math.toRadians(90))
                .splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)
                //.splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineToLinearHeading(new Pose2d(-54, -18, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .setReversed(true)
                //.splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(0)), Math.toRadians(180))
                .splineToLinearHeading(new Pose2d(-62, -12, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -18, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -6, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-48, -12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-30, -18, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(0)), Math.toRadians(180))
                .splineToSplineHeading(new Pose2d(-62, -12, Math.toRadians(0)), Math.toRadians(180))
                .setReversed(false)
                .waitSeconds(0.5)
                .splineTo(new Vector2d(-24, -12), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-6, -18, Math.toRadians(315)), Math.toRadians(315))
                .waitSeconds(0.5)
                .setReversed(true)
                //.splineToSplineHeading(new Pose2d(-48, -12, Math.toRadians(90)), Math.toRadians(180))
                //.splineToSplineHeading(new Pose2d(-60, -24, Math.toRadians(90)), Math.toRadians(270))
                //.splineToSplineHeading(new Pose2d(-60, -36, Math.toRadians(90)), Math.toRadians(270))
                //.splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(90)), Math.toRadians(270))
                //.splineToSplineHeading(new Pose2d(-24, -12, Math.toRadians(90)), Math.toRadians(0))
                .splineToSplineHeading(new Pose2d(-12, -24, Math.toRadians(90)), Math.toRadians(270))
                .splineToSplineHeading(new Pose2d(-12, -36, Math.toRadians(90)), Math.toRadians(270))
                .build();

        drive.followTrajectorySequenceAsync(trajSeq);

        waitForStart();

        while (opModeIsActive()) {
            drive.update();
            dashboardTelemetry.update();
        }
    }
}