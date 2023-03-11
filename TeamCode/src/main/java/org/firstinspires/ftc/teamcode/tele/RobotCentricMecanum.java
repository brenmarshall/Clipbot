package org.firstinspires.ftc.teamcode.tele;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController;

// Warning: this code is garbage and nobody knows how it works :D
@TeleOp(name="RobotCentric")
public class RobotCentricMecanum extends LinearOpMode {

    double vertPow, gripPos;
    double multiplier = 1.0;

    public static double Kp = 0.005, Ki = 0, Kd = 0;
    public static int targetInches = 0;

    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    DcMotorEx liftEncoder;
    CRServo liftMotor;

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

        liftEncoder = hardwareMap.get(DcMotorEx.class, "motorFrontRight");
        liftMotor = hardwareMap.crservo.get("vertical"); // Ensure Spark Mini is on Braking

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Declare our motors
        // Make sure your ID's match your configuration
        DcMotor motorLeft = hardwareMap.dcMotor.get("motorLeft");
        DcMotor motorRight = hardwareMap.dcMotor.get("motorRight");

        telemetry.addLine("Ready");
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            if(gamepad1.left_trigger > 0.2 || gamepad1.right_trigger > 0.2 || gamepad1.left_bumper || gamepad1.right_bumper) {
                multiplier = 0.5;
            }
            else {
                multiplier = 1.0;
            }

            Boolean manual = false;

            Boolean rBump = gamepad2.right_bumper;
            Boolean lBump = gamepad2.left_bumper;

            double rTrigger = gamepad2.right_trigger;
            double lTrigger = gamepad2.left_trigger;

            // Gripper
            if (gamepad2.a) {
                gripServo.setPosition(0.875);
                leftV4B.setPosition(0.0);
                rightV4B.setPosition(0.0);
                leftGuide.setPosition(0.0);
                rightGuide.setPosition(0.0);
            }

            if (gamepad2.b) {
                leftV4B.setPosition(1.0);
                rightV4B.setPosition(1.0);
                leftGuide.setPosition(1.0);
                rightGuide.setPosition(1.0);
            }

            // Guide
            if (gamepad2.x) {
                leftGuide.setPosition(0.33);
            } else if (gamepad2.y) {
                leftGuide.setPosition(0.0);
            }

            // Auto heights
            if (gamepad2.dpad_up) {
                targetInches = 38;
            } else if (gamepad2.dpad_right) {
                targetInches = 28;
            } else if (gamepad2.dpad_down) {
                targetInches = 18;
            } else if (gamepad2.dpad_left) {
                targetInches = 1;
            }


            // Vertical Slides
            if (manual) {
                if (rTrigger > 0.2) {
                    vertPow = -1.0;
                }
                else if (rBump) {
                    vertPow = 1.0;
                }
                else if (lTrigger > 0.2) {
                    vertPow = -0.5;
                }
                else if (lBump) {
                    vertPow = 0.5;
                }
                else {
                    vertPow = 0.0;
                }
                int targetPosition = (int)(targetInches * 30.72);
                control.update(targetPosition, liftEncoder.getCurrentPosition());

                liftMotor.setPower(vertPow);
            }
            else {
                int targetPosition = (int)(targetInches * 30.72);
                // Update pid controller
                double command = control.update(targetPosition, liftEncoder.getCurrentPosition());
                command = Range.clip(command, -1, 1);
                // Assign PID output
                dashboardTelemetry.addData("Command", command);
                liftMotor.setPower(command);
            }

            // drivePower is the power for forward/backward movement
            // rotatePower is the power for rotating the robot
            float drivePower = -gamepad1.left_stick_y;
            float rotatePower = gamepad1.right_stick_x;

            // Flip these signs if the robot rotates the wrong way
            motorLeft.setPower(drivePower + rotatePower);
            motorRight.setPower(drivePower - rotatePower);

            telemetry.update();
            dashboardTelemetry.update();
        }
    }
}
