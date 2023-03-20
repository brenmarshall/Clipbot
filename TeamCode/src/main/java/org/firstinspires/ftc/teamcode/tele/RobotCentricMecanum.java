package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.util.PIDController;

// Warning: this code is garbage and nobody knows how it works :D
@TeleOp(name="RobotCentricMecanum")
public class RobotCentricMecanum extends LinearOpMode {

    public static double Kp = 0.005, Ki = 0.0, Kd = 0.0;
    public static double targetInches = 0.0;
    public static double low = 1;
    public static double medium = 11;
    public static double high = 21;
    public static double open = 0.8;
    public static double closed = 1.0;
    double multiplier = 1.0;
    double stackHeight = 0.0;

    NormalizedColorSensor clawSensor;
    NormalizedColorSensor guideSensor;
    DistanceSensor clawDistanceSensor;
    DistanceSensor guideDistanceSensor;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    PIDController control = new PIDController(Kp, Ki, Kd, dashboardTelemetry);

    @Override
    public void runOpMode() throws InterruptedException {

        // FTC Dashboard Setups
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();

        // Servo
        Servo gripServo = hardwareMap.servo.get("manipulator");
        Servo leftV4B = hardwareMap.servo.get("leftV4B");
        Servo rightV4B = hardwareMap.servo.get("rightV4B");
        Servo leftGuide = hardwareMap.servo.get("leftGuide");
        Servo rightGuide = hardwareMap.servo.get("rightGuide");

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor liftMotorLeft = hardwareMap.dcMotor.get("liftMotorLeft");
        DcMotor liftMotorRight = hardwareMap.dcMotor.get("liftMotorRight");
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("motorFrontLeft");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("motorBackLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("motorFrontRight");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("motorBackRight");

        // Reverse the motors
        motorFrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        clawSensor = hardwareMap.get(NormalizedColorSensor.class, "clawSensor");
        guideSensor = hardwareMap.get(NormalizedColorSensor.class, "guideSensor");
        clawDistanceSensor = hardwareMap.get(DistanceSensor.class, "clawSensor");
        guideDistanceSensor = hardwareMap.get(DistanceSensor.class, "guideSensor");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            NormalizedRGBA clawColors = clawSensor.getNormalizedColors();
            NormalizedRGBA guideColors = guideSensor.getNormalizedColors();
            double clawDistance = clawDistanceSensor.getDistance(DistanceUnit.MM);
            double guideDistance = guideDistanceSensor.getDistance(DistanceUnit.MM);
            if(clawColors.red > 0.9 && clawColors.green < 0.1 && clawColors.blue < 0.1 && clawDistance < 20 || clawColors.red < 0.1 && clawColors.green < 0.1 && clawColors.blue > 0.9 && clawDistance < 20) {
                gripServo.setPosition(closed);
            }

            if(guideColors.red > 0.9 && guideColors.green > 0.9  && guideColors.blue < 0.1 && guideDistance < 10 && targetInches >= 1) {
                targetInches = targetInches - 1;
            }

            if(gamepad1.left_trigger > 0.2) {
                multiplier = 0.5;
            }
            else if(targetInches < 10) {
                multiplier = 0.75;
            }
            else {
                multiplier = 1.0;
            }

            // Release cone
            if (gamepad1.x) {
                if (targetInches >= 1) {
                    targetInches = targetInches - 1;
                    gripServo.setPosition(open);
                    leftV4B.setPosition(0.0);
                    rightV4B.setPosition(0.83);
                    leftGuide.setPosition(0.0);
                    rightGuide.setPosition(0.3);
                    targetInches = 0;
                } else {
                    gripServo.setPosition(open);
                }
            }
            // Manual claw
            if (gamepad1.dpad_right) {
                gripServo.setPosition(closed);
            }

            // Guide
            if (gamepad1.dpad_left) {
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.dpad_up) {
                leftGuide.setPosition(0.3);
                rightGuide.setPosition(0.0);
            }

            // Auto heights
            if (gamepad1.y) {
                gripServo.setPosition(closed);
                targetInches = high;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.b) {
                gripServo.setPosition(closed);
                targetInches = medium;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.a) {
                gripServo.setPosition(closed);
                targetInches = low;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.dpad_down) {
                gripServo.setPosition(open);
                targetInches = 0;
                leftV4B.setPosition(0.0);
                rightV4B.setPosition(0.83);
                leftGuide.setPosition(0.3);
                rightGuide.setPosition(0.0);
            }
            // Cone stack heights
            if (gamepad1.right_trigger > 0.2 && targetInches >= 1) {
                stackHeight = stackHeight - 1;
                targetInches = stackHeight;
            }

            if (gamepad1.right_bumper) {
                stackHeight = stackHeight + 1;
                targetInches = stackHeight;
            }

            int targetPosition = (int)(targetInches * 30.72);
            // Update pid controller
            double leftCommand = control.update(targetPosition, liftMotorLeft.getCurrentPosition());
            double rightCommand = control.update(targetPosition, liftMotorRight.getCurrentPosition());
            leftCommand = Range.clip(leftCommand, -1, 1);
            rightCommand = Range.clip(rightCommand, -1, 1);
            // Assign PID output
            dashboardTelemetry.addData("Command", leftCommand);
            dashboardTelemetry.addData("Command", rightCommand);
            liftMotorLeft.setPower(leftCommand);
            liftMotorLeft.setPower(rightCommand);

            double y = -gamepad1.left_stick_y * multiplier; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * 1.1 * multiplier; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            packet.put("frontLeftPower", frontLeftPower);
            packet.put("frontRightPower", frontRightPower);
            packet.put("backLeftPower", backLeftPower);
            packet.put("backRightPower", backRightPower);

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

            telemetry.update();
            dashboardTelemetry.update();
        }
    }
}
