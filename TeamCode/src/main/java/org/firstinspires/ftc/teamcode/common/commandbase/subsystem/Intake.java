package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.util.ServoFunctions;

public class Intake extends SubsystemBase {

    private final Bot bot;
    private final DcMotorEx intakeSlidesMotor; // bare motor HAVE
    private final CRServo turretServo; // axon mini/agfrc sa33 NEED
    private final AnalogInput turretEncoder; // custom mt6701 board NEED
    private final Servo intakeArmServo; // gb speed NEED
    private final Servo intakeWristServo; // gb speed NEED
    private final Servo intakeClawServo; // agfrc sa30 NEED

    private final PIDFController intakeSlidesController;
    private final PIDFController turretController;

    private double targetIntakeSlidesPosition = 0.0;
    private double targetTurretAngle = 0.0;

    private final double INTAKE_ARM_SERVO_RANGE = 270.0;
    private final double INTAKE_ARM_MIN_ANGLE = 0.0;
    private final double INTAKE_ARM_MAX_ANGLE = 115.0;
    private final double INTAKE_WRIST_SERVO_RANGE = 270.0;
    private final double INTAKE_WRIST_MIN_ANGLE = -90.0;
    private final double INTAKE_WRIST_MAX_ANGLE = 90.0;

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
                Configuration.intakeSlides_kP, // kP
                Configuration.intakeSlides_kI, // kI
                Configuration.intakeSlides_kD, // kD
                Configuration.intakeSlides_kF  // kF
        );

        turretController = new PIDFController(
                Configuration.turret_kP, // kP
                Configuration.turret_kI, // kI
                Configuration.turret_kD, // kD
                Configuration.turret_kF  // kF
        );
    }

    public void periodic()  {
        double intakeSlidesPower = intakeSlidesController.calculate(
                intakeSlidesMotor.getCurrentPosition(),
                targetIntakeSlidesPosition * Configuration.intakeSlides_ticksPerCM
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
        return intakeSlidesMotor.getCurrentPosition() / Configuration.intakeSlides_ticksPerCM;
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

    public void setIntakeArmAngle(double angle) {
        intakeArmServo.setPosition(ServoFunctions.degreesToServoPosition((ServoFunctions.clampAngleDegrees(angle, INTAKE_ARM_MIN_ANGLE, INTAKE_ARM_MAX_ANGLE)), INTAKE_ARM_SERVO_RANGE));
    }

    public void setIntakeWristAngle(double angle) {
        intakeWristServo.setPosition(ServoFunctions.degreesToServoPosition((ServoFunctions.clampAngleDegrees(angle, INTAKE_WRIST_MIN_ANGLE, INTAKE_WRIST_MAX_ANGLE)), INTAKE_WRIST_SERVO_RANGE));
    }

    public void setIntakeClawPosition(double position) {
        intakeClawServo.setPosition(position);
    }
}
