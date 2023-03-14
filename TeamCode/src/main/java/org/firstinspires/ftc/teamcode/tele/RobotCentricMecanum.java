package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

// Warning: this code is garbage and nobody knows how it works :D
@TeleOp(name="RobotCentricTank")
public class RobotCentricMecanum extends LinearOpMode {

    double multiplier = 1.0;

    public static double Kp = 0.005, Ki = 0, Kd = 0;
    public static double targetInches = 0.0;
    public static int stackHeight = 0;

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

        DcMotor driveMotorLeft = hardwareMap.dcMotor.get("driveMotorLeft");
        DcMotor driveMotorRight = hardwareMap.dcMotor.get("driveMotorRight");

        driveMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0.2) {
                multiplier = 0.5;
            }
            else if(targetInches < 10) {
                multiplier = 0.5;
            }
            else {
                multiplier = 1.0;
            }

            // Release cone
            if (gamepad1.x) {
                if (targetInches >= 1) {
                    targetInches = targetInches - 1;
                    gripServo.setPosition(0.8);
                    leftV4B.setPosition(0.0);
                    rightV4B.setPosition(0.83);
                    leftGuide.setPosition(0.0);
                    rightGuide.setPosition(0.3);
                    targetInches = 0;
                } else {
                    gripServo.setPosition(0.8);
                }
            }
            // Manual claw
            if (gamepad1.dpad_right) {
                gripServo.setPosition(1.0);
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
                gripServo.setPosition(1.0);
                targetInches = 21;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.b) {
                gripServo.setPosition(1.0);
                targetInches = 11;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.a) {
                gripServo.setPosition(1.0);
                targetInches = 1;
                leftV4B.setPosition(0.83);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.3);
            } else if (gamepad1.dpad_down) {
                gripServo.setPosition(0.8);
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
