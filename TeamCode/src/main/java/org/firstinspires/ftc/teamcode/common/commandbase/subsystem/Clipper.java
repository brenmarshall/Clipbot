package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

public class Clipper extends SubsystemBase {
    private final Bot bot;
    private final DcMotorEx clipperDriveMotor;
    private final DigitalChannel clipperLimitSwitch;
    private final Servo clipperServo;
    private final Servo clipmagPivotServo;
    private final Servo clipmagServo;

    private final PIDFController clipperDriveController;

    private double targetClipperPosition = 0.0;
    public final double clipWidth = 2.54; // cm
    public int magPosition = 0;
    private boolean isHoming = true;

    public Clipper(Bot bot) {
        this.bot = bot;

        clipperDriveMotor = bot.hMap.get(DcMotorEx.class, "clipperDriveMotor");
        clipperLimitSwitch = bot.hMap.get(DigitalChannel.class, "clipperLimitSwitch");
        clipperServo = bot.hMap.get(Servo.class, "clipperServo");
        clipmagPivotServo = bot.hMap.get(Servo.class, "clipmagPivotServo");
        clipmagServo = bot.hMap.get(Servo.class, "clipmagServo");

        clipperDriveMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        clipperDriveMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        clipperDriveController = new PIDFController(
                Config.clipperDrive_kP, // kP
                Config.clipperDrive_kI, // kI
                Config.clipperDrive_kD, // kD
                Config.clipperDrive_kF  // kF
        );
    }

    public void periodic() {
        if (!isHoming) {
            double clipperDrivePower = clipperDriveController.calculate(
                    clipperDriveMotor.getCurrentPosition(),
                    targetClipperPosition * Config.clipperDrive_ticksPerCM
            );

            clipperDriveMotor.setPower(clipperDrivePower);
        }

    }

    public void setClipperDrivePosition(double target) {
        targetClipperPosition = target;
    }

    public double getClipperDrivePosition() {
        return clipperDriveMotor.getCurrentPosition() / Config.clipperDrive_ticksPerCM;
    }

    public void setClipperDrivePower(double power) {
        clipperDriveMotor.setPower(power);
    }

    public boolean isClipperLimitSwitchPressed() {
        return !clipperLimitSwitch.getState();
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

    public void setClipperServoPosition(double position) {
        clipperServo.setPosition(position);
    }

    public void setClipmagPivotServoPosition(double position) {
        clipmagPivotServo.setPosition(position);
    }

    public void setClipmagServoPosition(double position) {
        clipmagServo.setPosition(position);
    }
}
