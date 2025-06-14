package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;

public class Deposit extends SubsystemBase {
    private final Bot bot;
    private final DcMotorEx leftDepositSlidesMotor; // bare motor HAVE
    private final DcMotorEx rightDepositSlidesMotor; // bare motor HAVE
    private final Servo DepositArmServo; // axon mini/agfrc sa33 NEED
    private final Servo DepositWristServo; // gb torque NEED
    private final Servo DepositClawServo; // agfrc sa30 NEED

    private final PIDFController depositSlidesController;

    private double targetDepositSlidesPosition = 0.0;

    public Deposit(Bot bot) {
        this.bot = bot;
        leftDepositSlidesMotor = bot.hMap.get(DcMotorEx.class, "leftDepositSlidesMotor");
        rightDepositSlidesMotor = bot.hMap.get(DcMotorEx.class, "rightDepositSlidesMotor");
        DepositArmServo = bot.hMap.get(Servo.class, "depositArmServo");
        DepositWristServo = bot.hMap.get(Servo.class, "depositWristServo");
        DepositClawServo = bot.hMap.get(Servo.class, "depositClawServo");

        leftDepositSlidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightDepositSlidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftDepositSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightDepositSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        depositSlidesController = new PIDFController(
                Configuration.depositSlides_kP, // kP
                Configuration.depositSlides_kI, // kI
                Configuration.depositSlides_kD, // kD
                Configuration.depositSlides_kF  // kF
        );
    }

    public void periodic() {
        double depositSlidesPower = depositSlidesController.calculate(
                leftDepositSlidesMotor.getCurrentPosition(),
                targetDepositSlidesPosition * Configuration.depositSlides_ticksPerCM
        );

        leftDepositSlidesMotor.setPower(depositSlidesPower);
        rightDepositSlidesMotor.setPower(depositSlidesPower);
    }

    public void setDepositSlidesPosition(double target) {
        targetDepositSlidesPosition = target;
    }

    public double getDepositSlidesPosition() {
        return leftDepositSlidesMotor.getCurrentPosition() / Configuration.depositSlides_ticksPerCM;
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
