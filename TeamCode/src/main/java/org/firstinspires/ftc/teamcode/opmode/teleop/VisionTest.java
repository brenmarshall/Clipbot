package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.vision.CVMaster;

import java.util.Arrays;

@TeleOp
public class VisionTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Bot.dashboard = com.acmerobotics.dashboard.FtcDashboard.getInstance();
        CVMaster.isHsvTest = true;
        CVMaster vision = new CVMaster(this);
        waitForStart();
        while (opModeIsActive()) {
            telemetry.addData("Center", Arrays.toString(vision.getCenter()));
            telemetry.addData("Color", vision.getCurrent());
            telemetry.update();
            sleep(100);
        }
    }
}