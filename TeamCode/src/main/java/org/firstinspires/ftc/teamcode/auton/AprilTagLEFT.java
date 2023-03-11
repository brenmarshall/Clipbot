package org.firstinspires.ftc.teamcode.auton;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.util.PIDController;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;


// Credit: OpenFTC for a lot
@Config
@Autonomous(name="Parking LEFT SIDE")
public class AprilTagLEFT extends LinearOpMode
{
    public static double Kp = 0.005, Ki = 0, Kd = 0;
    public static int targetInches = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    PoleObserverPipeline poleObserverPipeline;

    int strafeDistance = 0;

    static final double FEET_PER_METER = 3.28084;

    // Lens intrinsics
    // UNITS ARE PIXELS
    // NOTE: this calibration is for the C920 webcam at 800x448.
    // You will need to do your own calibration for other configurations!
    double fx = 578.272;
    double fy = 578.272;
    double cx = 402.145;
    double cy = 221.506;

    // UNITS ARE METERS
    double tagsize = 0.166;

    // Tag ID 1,2,3 from the 36h11 family
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;

    AprilTagDetection tagOfInterest = null;

    @Override
    public void runOpMode()
    {
        telemetry.setAutoClear(true);
        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();

        SampleTankDrive drive = new SampleTankDrive(hardwareMap);
        Pose2d startPose = new Pose2d(-36, -63, Math.toRadians(90)); // Set start pose to center of the field, facing north
        drive.setPoseEstimate(startPose);

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        Servo guide = hardwareMap.servo.get("guide");
        Servo gripServo = hardwareMap.servo.get("manipulator");
        Servo signal = hardwareMap.servo.get("signal");


        liftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        poleObserverPipeline = new PoleObserverPipeline(telemetry);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(camera,5);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });

        telemetry.setMsTransmissionInterval(50);

        TrajectorySequence parkingOne = drive.trajectorySequenceBuilder(startPose)
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
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                // Parking spot 1
                .splineTo(new Vector2d(-48, -36), Math.toRadians(180))
                .splineTo(new Vector2d(-60, -36), Math.toRadians(270))
                .build();

        TrajectorySequence parkingTwo = drive.trajectorySequenceBuilder(startPose)
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
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                // Parking spot 2
                .splineTo(new Vector2d(-36, -36), Math.toRadians(270))
                .build();

        TrajectorySequence parkingThree = drive.trajectorySequenceBuilder(startPose)
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
                .splineTo(new Vector2d(-36, -24), Math.toRadians(270))
                // Parking spot 3
                .splineTo(new Vector2d(-24, -36), Math.toRadians(0))
                .splineTo(new Vector2d(-12, -36), Math.toRadians(270))
                .build();

        //drive.followTrajectorySequenceAsync(trajSeq);

        /*
         * The INIT-loop:
         * This REPLACES waitForStart!
         */
        while (!isStarted() && !isStopRequested())
        {
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

            if(currentDetections.size() != 0)
            {
                boolean tagFound = false;

                for(AprilTagDetection tag : currentDetections)
                {
                    if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT)
                    {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }

                if(tagFound)
                {
                    telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                    tagToTelemetry(tagOfInterest);
                    tagToPacket(tagOfInterest, packet);
                }
                else
                {
                    telemetry.addLine("Don't see tag of interest :(");

                    if(tagOfInterest == null)
                    {
                        telemetry.addLine("(The tag has never been seen)");
                    }
                    else
                    {
                        telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                        tagToTelemetry(tagOfInterest);
                    }
                }

            }
            else
            {
                telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null)
                {
                    telemetry.addLine("(The tag has never been seen)");
                }
                else
                {
                    telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    tagToTelemetry(tagOfInterest);
                }

            }

            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
            sleep(20);
        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */
        camera.setPipeline(poleObserverPipeline);

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
            telemetry.addLine("Tag snapshot:\n");
            tagToTelemetry(tagOfInterest);
            telemetry.update();
        }
        else
        {
            telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
            telemetry.update();
        }

        /* Actually do something useful */
        //region MOVEMENT
        // speed 0.4 is pretty good

        if(tagOfInterest == null || tagOfInterest.id == LEFT){
            drive.followTrajectorySequenceAsync(parkingOne);
        }else if(tagOfInterest.id == MIDDLE){
            drive.followTrajectorySequenceAsync(parkingTwo);
        }else{
            drive.followTrajectorySequenceAsync(parkingThree);
        }

        while(opModeIsActive()) {
            drive.update();

            int targetPosition = (int) (targetInches * 64.68056888);
            // Update pid controller
            double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
            command = Range.clip(command, -1, 1);
            // Assign PID output
            dashboardTelemetry.addData("Command", command);
            liftMotor.setPower(command);

            dashboardTelemetry.update();
        }
        //endregion


        /* You wouldn't have this in your autonomous, this is just to prevent the sample from ending */
        //while (opModeIsActive()) {sleep(20);}
    }

    @SuppressLint("DefaultLocale")
    void tagToTelemetry(AprilTagDetection detection)
    {
        telemetry.addLine(String.format("\nDetected tag ID=%d", detection.id));
        telemetry.addLine(String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        telemetry.addLine(String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
        telemetry.addLine(String.format("Rotation Yaw: %.2f degrees", Math.toDegrees(detection.pose.yaw)));
        telemetry.addLine(String.format("Rotation Pitch: %.2f degrees", Math.toDegrees(detection.pose.pitch)));
        telemetry.addLine(String.format("Rotation Roll: %.2f degrees", Math.toDegrees(detection.pose.roll)));
    }

    @SuppressLint("DefaultLocale")
    void tagToPacket(AprilTagDetection detection, TelemetryPacket packet)
    {
        packet.put("Detected Tag", String.format("Detected tag ID=%d", detection.id));
        packet.put("Translation X", String.format("Translation X: %.2f feet", detection.pose.x*FEET_PER_METER));
        packet.put("Translation Y", String.format("Translation Y: %.2f feet", detection.pose.y*FEET_PER_METER));
        packet.put("Translation Z", String.format("Translation Z: %.2f feet", detection.pose.z*FEET_PER_METER));
    }

}