package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

public class Deposit extends SubsystemBase {
    private final Bot bot;
    private final DcMotorEx depositSlidesMotor;
    private final Servo DepositArmServo;
    private final Servo DepositWristServo;
    private final Servo DepositClawServo;

    private final PIDFController depositSlidesController;

    private double targetDepositSlidesPosition = 0.0;

    public Deposit(Bot bot) {
        this.bot = bot;
        depositSlidesMotor = bot.hMap.get(DcMotorEx.class, "depositSlidesMotor");
        DepositArmServo = bot.hMap.get(Servo.class, "DepositArmServo");
        DepositWristServo = bot.hMap.get(Servo.class, "DepositWristServo");
        DepositClawServo = bot.hMap.get(Servo.class, "DepositClawServo");

        depositSlidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        depositSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        depositSlidesController = new PIDFController(
                Config.depositSlides_kP, // kP
                Config.depositSlides_kI, // kI
                Config.depositSlides_kD, // kD
                Config.depositSlides_kF  // kF
        );
    }

    public void periodic() {
        double depositSlidesPower = depositSlidesController.calculate(
                depositSlidesMotor.getCurrentPosition(),
                targetDepositSlidesPosition * Config.depositSlides_ticksPerCM
        );

        depositSlidesMotor.setPower(depositSlidesPower);
    }

    public void setDepositSlidesPosition(double target) {
        targetDepositSlidesPosition = target;
    }

    public void setDepositArmPosition(double position) {
        DepositArmServo.setPosition(position);
    }

    public void setDepositWristPosition(double position) {
        DepositWristServo.setPosition(position);
    }

    public void setDepositClawPosition(double position) {
        DepositClawServo.setPosition(position);
    }
}
