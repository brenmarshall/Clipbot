package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

public class Intake extends SubsystemBase {

    private final Bot bot;
    private final DcMotorEx intakeSlidesMotor; // bare motor
    private final CRServo turretServo; // axon mini/agfrc sa33
    private final AnalogInput turretEncoder; // custom mt6701 board
    private final Servo intakeArmServo; // axon mini/agfrc sa33 or gb torque
    private final Servo intakeWristServo; // gb speed
    private final Servo intakeClawServo; // axon micro/agfrc sa30

    private final PIDFController intakeSlidesController;
    private final PIDFController turretController;

    private double targetIntakeSlidesPosition = 0.0;
    private double targetTurretAngle = 0.0;

    public Intake(Bot bot) {
        this.bot = bot;
        intakeSlidesMotor = bot.hMap.get(DcMotorEx.class, "intakeSlidesMotor");
        turretServo = bot.hMap.get(CRServo.class, "turretServo");
        turretEncoder = bot.hMap.get(AnalogInput.class, "turretEncoder");
        intakeArmServo = bot.hMap.get(Servo.class, "intakeArmServo");
        intakeWristServo = bot.hMap.get(Servo.class, "intakeWristServo");
        intakeClawServo = bot.hMap.get(Servo.class, "intakeClawServo");

        intakeSlidesMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlidesMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeSlidesController = new PIDFController(
                Config.intakeSlides_kP, // kP
                Config.intakeSlides_kI, // kI
                Config.intakeSlides_kD, // kD
                Config.intakeSlides_kF  // kF
        );

        turretController = new PIDFController(
                Config.turret_kP, // kP
                Config.turret_kI, // kI
                Config.turret_kD, // kD
                Config.turret_kF  // kF
        );
    }

    public void periodic()  {
        double intakeSlidesPower = intakeSlidesController.calculate(
                intakeSlidesMotor.getCurrentPosition(),
                targetIntakeSlidesPosition * Config.intakeSlides_ticksPerCM
        );

        intakeSlidesMotor.setPower(intakeSlidesPower);

        double intakeTurretPower = turretController.calculate(
                getTurretDegrees(),
                targetTurretAngle
        );

        turretServo.setPower(intakeTurretPower);
    }

    public void setIntakeSlidesPosition(double position) {
        targetIntakeSlidesPosition = position;
    }

    public double getIntakeSlidesPosition() {
        return intakeSlidesMotor.getCurrentPosition() / Config.intakeSlides_ticksPerCM;
    }

    private double mapVoltageToDegrees(double voltage) {
        double maxVoltage = 3.3;
        return (voltage / maxVoltage) * 360;
    }

    public double getTurretDegrees() {
        double feedbackVoltage = turretEncoder.getVoltage();
        return mapVoltageToDegrees(feedbackVoltage);
    }

    public void setTurretAngle(double angle) {
        targetTurretAngle = angle;
    }

    public void setIntakeArmPosition(double position) {
        intakeArmServo.setPosition(position);
    }

    public void setIntakeWristPosition(double position) {
        intakeWristServo.setPosition(position);
    }

    public void setIntakeClawPosition(double position) {
        intakeClawServo.setPosition(position);
    }
}
