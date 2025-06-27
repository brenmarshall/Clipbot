package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.util.ServoFunctions;

public class Intake extends SubsystemBase {

    private final Bot bot;
    private final DcMotorEx intakeSlidesMotor; // bare motor HAVE
    private final CRServo turretServo; // axon mini/agfrc sa33 NEED
    private final AnalogInput turretEncoder; // custom mt6701 board NEED
    private final ServoImplEx intakeArmServo; // 299:lp HAVE
    private final Servo intakeWristServo; // gb speed HAVE
    private final Servo intakeClawServo; // raw 100 mini HAVE
    private final DigitalChannel intakeClawSensor; // custom ina169 board NEED
    private final AnalogInput intakeSubSensor; // pololu GP2Y0A60SZLF HAVE

    private final PIDFController intakeSlidesController;
    private final PIDFController turretController;

    private double targetIntakeSlidesPosition = 0.0;
    private double targetTurretAngle = 0.0;
    private double targetIntakeArmPosition = 0.0;

    private final double INTAKE_ARM_SERVO_RANGE = 180.0;
    private final double INTAKE_ARM_MIN_ANGLE = 0.0;
    private final double INTAKE_ARM_MAX_ANGLE = 115.0;
    private final double INTAKE_WRIST_SERVO_RANGE = 270.0;
    private final double INTAKE_WRIST_MIN_ANGLE = -90.0;
    private final double INTAKE_WRIST_MAX_ANGLE = 90.0;

    public enum IntakeClawState {
        OPEN,
        HOLD,
        CLOSED
    }

    public IntakeClawState intakeClawState;

    public Intake(Bot bot) {
        this.bot = bot;
        intakeSlidesMotor = bot.hMap.get(DcMotorEx.class, "intakeSlidesMotor");
        turretServo = bot.hMap.get(CRServo.class, "turretServo");
        turretEncoder = bot.hMap.get(AnalogInput.class, "turretEncoder");
        intakeArmServo = bot.hMap.get(ServoImplEx.class, "intakeArmServo");
        intakeWristServo = bot.hMap.get(Servo.class, "intakeWristServo");
        intakeClawServo = bot.hMap.get(Servo.class, "intakeClawServo");
        intakeClawSensor = bot.hMap.get(DigitalChannel.class, "intakeClawSensor");
        intakeSubSensor = bot.hMap.get(AnalogInput.class, "intakeSubSensor");

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

        intakeArmServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArmServo.setDirection(Servo.Direction.REVERSE);
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

        bot.telem.addData("Intake Sub Sensor Voltage", intakeSubSensor.getVoltage());
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

    public void setIntakeArmPosition(double position) {
        intakeArmServo.setPosition(position);
        targetIntakeArmPosition = position;
    }

    public void incrementIntakeArmPosition(double increment) {
        double currentPosition = intakeArmServo.getPosition();
        double newPosition = currentPosition + increment;
        intakeArmServo.setPosition(newPosition);
    }

    public void setIntakeWristAngle(double angle) {
        intakeWristServo.setPosition(ServoFunctions.degreesToServoPosition((ServoFunctions.clampAngleDegrees(angle, INTAKE_WRIST_MIN_ANGLE, INTAKE_WRIST_MAX_ANGLE)), INTAKE_WRIST_SERVO_RANGE));
    }

    public void setIntakeClawPosition(IntakeClawState state) {
        switch (state) {
            case OPEN:
                intakeClawServo.setPosition(Configuration.intakeClawOpen);
                break;
            case HOLD:
                intakeClawServo.setPosition(Configuration.intakeClawHold);
                break;
            case CLOSED:
                intakeClawServo.setPosition(Configuration.intakeClawClose);
                break;
        }
    }

    public void setIntakeClawState(IntakeClawState state) {
        intakeClawState = state;
        setIntakeClawPosition(state);
    }

    public boolean isIntakeClawGrabbing() {
        switch (intakeClawState) {
            case OPEN:
                return false;
            case CLOSED:
                return intakeClawSensor.getState();
            default:
                return false;
        }
    }
}
