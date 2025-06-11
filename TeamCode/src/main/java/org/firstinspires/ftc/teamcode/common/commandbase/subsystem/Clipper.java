package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;

public class Clipper extends SubsystemBase {
    private final Bot bot;
    private final DcMotorEx clipperDriveMotor; // bare motor HAVE
    private final DigitalChannel clipperLimitSwitch; // limit switch HAVE
    private final Servo clipperServo; // gb speed HAVE
    private final Servo clipmagPivotServo; // gb torque HAVE
    private final Servo clipmagServo; // agfrc sa30 NEED

    private final PIDFController clipperDriveController;

    private double targetClipperPosition = 0.0;
    public final double clipWidth = 2.54; // cm
    public int magPosition = 7;
    private boolean isHoming = true;

    public enum ClipperState {
        CLIP,
        SET
    }

    public enum ClipMagState {
        UP,
        DOWN
    }

    public Clipper(Bot bot) {
        this.bot = bot;

        clipperDriveMotor = bot.hMap.get(DcMotorEx.class, "clipperDriveMotor");
        clipperLimitSwitch = bot.hMap.get(DigitalChannel.class, "clipperLimitSwitch");
        clipperServo = bot.hMap.get(Servo.class, "clipperServo");
        clipmagPivotServo = bot.hMap.get(Servo.class, "clipmagPivotServo");
        clipmagServo = bot.hMap.get(Servo.class, "clipmagServo");

        clipperDriveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clipperDriveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        clipperServo.setDirection(Servo.Direction.REVERSE);
        clipmagPivotServo.setDirection(Servo.Direction.REVERSE);

        clipperDriveController = new PIDFController(
                Configuration.clipperDrive_kP, // kP
                Configuration.clipperDrive_kI, // kI
                Configuration.clipperDrive_kD, // kD
                Configuration.clipperDrive_kF  // kF
        );
    }

    public void periodic() {
        if (!isHoming) {
            double clipperDrivePower = clipperDriveController.calculate(
                    clipperDriveMotor.getCurrentPosition(),
                    targetClipperPosition * Configuration.clipperDrive_ticksPerCM
            );

            clipperDriveMotor.setPower(clipperDrivePower);
        }

        bot.telem.addData("Clipper Limit Switch", isClipperLimitSwitchPressed() ? "Pressed" : "Not Pressed");
        bot.telem.addData("Is Homing", isHoming ? "Yes" : "No");
        bot.telem.addData("Clipper Drive Position", getClipperDrivePosition());
        bot.telem.addData("Target Clipper Position", targetClipperPosition);
        bot.telem.update();
    }

    public void setClipperDrivePosition(double target) {
        targetClipperPosition = target;
    }

    public double getClipperDrivePosition() {
        return clipperDriveMotor.getCurrentPosition() / Configuration.clipperDrive_ticksPerCM;
    }

    public void setClipperDrivePower(double power) {
        clipperDriveMotor.setPower(power);
    }

    public boolean isClipperLimitSwitchPressed() {
        return clipperLimitSwitch.getState();
    }

    public void setIsHoming(boolean isHoming) {
        this.isHoming = isHoming;
    }

    public void resetClipperEncoder() {
        clipperDriveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clipperDriveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void incrementMagPosition() {
        magPosition++;
    }

    public void setClipperPosition (ClipperState state) {
        switch (state) {
            case CLIP:
                clipperServo.setPosition(Configuration.clipperClipPosition);
                break;
            case SET:
                clipperServo.setPosition(Configuration.clipperSetPosition);
                break;
        }
    }

    public void setClipMagPosition(ClipMagState state) {
        switch (state) {
            case UP:
                clipmagPivotServo.setPosition(Configuration.clipMagUp);
                break;
            case DOWN:
                clipmagPivotServo.setPosition(Configuration.clipMagDown);
                break;
        }
    }

    public void setClipmagServoPosition(double position) {
        clipmagServo.setPosition(position);
    }
}
