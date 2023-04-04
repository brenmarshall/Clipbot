package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

// Warning: this code is garbage and nobody knows how it works :D
@TeleOp(name="RobotCentricTank")
public class RobotCentricTank extends LinearOpMode {

    public static double low = 1.0 * 30.71283;
    public static double medium = 11.0 * 30.71283;
    public static double high = 21.0 * 30.71283;
    public static double open = 0.8;
    public static double closed = 1.0;
    double multiplier = 1.0;
    double stackHeight = 0.0;

    //NormalizedColorSensor clawSensor;
    //NormalizedColorSensor guideSensor;
    //DistanceSensor clawDistanceSensor;
    //DistanceSensor guideDistanceSensor;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void runOpMode() throws InterruptedException {

        // FTC Dashboard Setups
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Servo
        Servo gripServo = hardwareMap.servo.get("manipulator");
        //Servo leftV4B = hardwareMap.servo.get("leftV4B");
        //Servo rightV4B = hardwareMap.servo.get("rightV4B");
        //Servo leftGuide = hardwareMap.servo.get("leftGuide");
        //Servo rightGuide = hardwareMap.servo.get("rightGuide");

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor driveMotorLeft = hardwareMap.dcMotor.get("driveMotorLeft");
        DcMotor driveMotorRight = hardwareMap.dcMotor.get("driveMotorRight");
        DcMotor liftMotorLeft = hardwareMap.dcMotor.get("liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.dcMotor.get("liftMotorRight");

        liftMotorLeft.setTargetPosition(0);
        liftMotorRight.setTargetPosition(0);

        liftMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        driveMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorRight.setDirection(DcMotorSimple.Direction.REVERSE);

        //clawSensor = hardwareMap.get(NormalizedColorSensor.class, "clawSensor");
        //guideSensor = hardwareMap.get(NormalizedColorSensor.class, "guideSensor");
        //clawDistanceSensor = hardwareMap.get(DistanceSensor.class, "clawSensor");
        //guideDistanceSensor = hardwareMap.get(DistanceSensor.class, "guideSensor");
        //clawSensor.setGain(10);

        gripServo.setPosition(open);

        Gamepad currentGamepad1 = new Gamepad();
        Gamepad previousGamepad1 = new Gamepad();

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            previousGamepad1.copy(currentGamepad1);
            currentGamepad1.copy(gamepad1);

            double position = (Math.abs(liftMotorLeft.getCurrentPosition()) + Math.abs(liftMotorRight.getCurrentPosition())) / (30.71283 / 2);
            telemetry.addData("Position", position);
            telemetry.addData("multiplier", multiplier);
            telemetry.addData("Stack Height", stackHeight);
            //NormalizedRGBA clawColors = clawSensor.getNormalizedColors();
            //NormalizedRGBA guideColors = guideSensor.getNormalizedColors();
            //double clawDistance = clawDistanceSensor.getDistance(DistanceUnit.MM);
            //double guideDistance = guideDistanceSensor.getDistance(DistanceUnit.MM);
            //if(clawColors.red > 0.2 && clawDistance < 20 || clawColors.blue > 0.2 && clawDistance < 20) {
                //gripServo.setPosition(closed);
                //telemetry.addData("Cone Detected", clawDistance);
            //}
            //else {
                //gripServo.setPosition(open);
            //}

            //if(guideColors.red > 0.9 && guideColors.green > 0.9  && guideColors.blue < 0.1 && guideDistance < 10 && targetInches >= 1) {
                //targetInches = targetInches - 1;
            //}

            if(gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2) {
                multiplier = 0.5;
            }
            else if(position > 12) {
                multiplier = 0.5;
            }
            else {
                multiplier = 1.0;
            }

            // Release cone
            if (gamepad1.x) {
                if (position >= 1) {
                    liftMotorLeft.setTargetPosition((int) position - 1);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition((int) position - 1);
                    liftMotorRight.setPower(1.0);
                    gripServo.setPosition(open);
                    //leftV4B.setPosition(0.0);
                    //rightV4B.setPosition(0.83);
                    //leftGuide.setPosition(0.0);
                    //rightGuide.setPosition(0.3);
                    liftMotorLeft.setTargetPosition(0);
                    liftMotorLeft.setPower(1.0);
                    liftMotorRight.setTargetPosition(0);
                    liftMotorRight.setPower(1.0);
                }
                else {
                    gripServo.setPosition(open);
                }
            }
            // Manual claw
            if (gamepad1.dpad_right) {
                gripServo.setPosition(closed);
            }

            // Guide
            //if (gamepad1.dpad_left) {
                //leftGuide.setPosition(0.0);
                //rightGuide.setPosition(0.3);
            //} else if (gamepad1.dpad_up) {
                //leftGuide.setPosition(0.3);
                //rightGuide.setPosition(0.0);
            //}

            // Auto heights
            if (gamepad1.y) {
                gripServo.setPosition(closed);
                liftMotorLeft.setTargetPosition((int) high);
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition((int) high);
                liftMotorRight.setPower(1.0);
                //leftV4B.setPosition(0.83);
                //rightV4B.setPosition(0.0);
                //leftGuide.setPosition(0.0);
                //rightGuide.setPosition(0.3);
            }
            else if (gamepad1.b) {
                gripServo.setPosition(closed);
                liftMotorLeft.setTargetPosition((int) medium);
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition((int) medium);
                liftMotorRight.setPower(1.0);
                //leftV4B.setPosition(0.83);
                //rightV4B.setPosition(0.0);
                //leftGuide.setPosition(0.0);
                //rightGuide.setPosition(0.3);
            }
            else if (gamepad1.a) {
                gripServo.setPosition(closed);
                liftMotorLeft.setTargetPosition((int) low);
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition((int) low);
                liftMotorRight.setPower(1.0);
                //leftV4B.setPosition(0.83);
                //rightV4B.setPosition(0.0);
                //leftGuide.setPosition(0.0);
                //rightGuide.setPosition(0.3);
            }
            else if (gamepad1.dpad_down) {
                liftMotorLeft.setTargetPosition(0);
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition(0);
                liftMotorRight.setPower(1.0);
                //leftV4B.setPosition(0.0);
                //rightV4B.setPosition(0.83);
                //leftGuide.setPosition(0.3);
                //rightGuide.setPosition(0.0);
            }
            // Cone stack heights
            if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && stackHeight >= 1) {
                stackHeight = stackHeight - 1;
                liftMotorLeft.setTargetPosition((int) (stackHeight * 30.71283));
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition((int) (stackHeight * 30.71283));
                liftMotorRight.setPower(1.0);
            }

            if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper) {
                stackHeight = stackHeight + 1;
                liftMotorLeft.setTargetPosition((int) (stackHeight * 30.71283));
                liftMotorLeft.setPower(1.0);
                liftMotorRight.setTargetPosition((int) (stackHeight * 30.71283));
                liftMotorRight.setPower(1.0);
            }

            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            // Flip these signs if the robot rotates the wrong way
            driveMotorLeft.setPower((drivePower + rotatePower) * multiplier);
            driveMotorRight.setPower((drivePower - rotatePower) * multiplier);

            telemetry.update();
            dashboardTelemetry.update();
        }
    }
}
