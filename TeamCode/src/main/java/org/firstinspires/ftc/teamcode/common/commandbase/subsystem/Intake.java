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
    private final CRServo intakeTurretServo; // axon mini/agfrc sa33 NEED
    private final AnalogInput intakeTurretEncoder; // custom mt6701 board NEED
    private final DigitalChannel intakeTurretLimitSwitch; // limit switch HAVE
    private final ServoImplEx intakeArmServo; // 299:lp HAVE
    private final ServoImplEx intakeWristServo; // gb speed HAVE
    private final Servo intakeClawServo; // raw 100 mini HAVE
    private final DigitalChannel intakeClawSensor; // custom ina169 board NEED
    private final AnalogInput intakeSubSensor; // pololu GP2Y0A60SZLF HAVE

    private final PIDFController intakeSlidesController;
    private final PIDFController intakeTurretController;

    private double targetIntakeSlidesPosition = 0.0;

    public double revolutionCounter = 0.0;
    public int servoRevolution = 0;
    public double previousEncoderAngle = 0.0;
    public double encoderZeroOffset = 0.0;

    public double currentTurretAngle = 0.0;
    public double targetTurretAngle = 0.0;
    private double targetIntakeArmPosition = 0.0;

    private final double INTAKE_TURRET_RATIO = 60.0 / 30.0;
    private final double INTAKE_TURRET_MIN_ANGLE = 20.0;
    private final double INTAKE_TURRET_MAX_ANGLE = 300.0;
    private final double INTAKE_ARM_SERVO_RANGE = 160.0;
    private final double INTAKE_ARM_RATIO = 40.0 / 48.0;
    private final double INTAKE_ARM_MIN_ANGLE = 0.0;
    private final double INTAKE_ARM_MAX_ANGLE = 115.0;
    private final double INTAKE_WRIST_SERVO_RANGE = 300.0;
    private final double INTAKE_WRIST_RATIO = 40.0 / 48.0;
    private final double INTAKE_WRIST_MIN_ANGLE = 0.0;
    private final double INTAKE_WRIST_MAX_ANGLE = 180.0;

    public enum IntakeTurretState {
        HOMING,
        POSITION_CONTROL,
        MANUAL_OVERRIDE
    }

    public enum IntakeClawState {
        OPEN,
        HOLD,
        CLOSED
    }

    public IntakeTurretState intakeTurretState = IntakeTurretState.HOMING;

    public IntakeClawState intakeClawState;

    public Intake(Bot bot) {
        this.bot = bot;
        intakeSlidesMotor = bot.hMap.get(DcMotorEx.class, "intakeSlidesMotor");
        intakeTurretServo = bot.hMap.get(CRServo.class, "intakeTurretServo");
        intakeTurretEncoder = bot.hMap.get(AnalogInput.class, "intakeTurretEncoder");
        intakeTurretLimitSwitch = bot.hMap.get(DigitalChannel.class, "intakeTurretLimitSwitch");
        intakeArmServo = bot.hMap.get(ServoImplEx.class, "intakeArmServo");
        intakeWristServo = bot.hMap.get(ServoImplEx.class, "intakeWristServo");
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

        intakeTurretController = new PIDFController(
                Configuration.turret_kP, // kP
                Configuration.turret_kI, // kI
                Configuration.turret_kD, // kD
                Configuration.turret_kF  // kF
        );

        intakeArmServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeArmServo.setDirection(Servo.Direction.REVERSE);
        intakeWristServo.setPwmRange(new PwmControl.PwmRange(500, 2500));
        intakeClawServo.setDirection(Servo.Direction.REVERSE);
    }

    public void periodic()  {

        double intakeSlidesPower = intakeSlidesController.calculate(
                intakeSlidesMotor.getCurrentPosition(),
                targetIntakeSlidesPosition * Configuration.intakeSlides_ticksPerCM
        );

        intakeSlidesMotor.setPower(intakeSlidesPower);

        updateAngleCalculation();

        switch (intakeTurretState) {
            case HOMING:

            case MANUAL_OVERRIDE:
                break;

            case POSITION_CONTROL:
                double turretPower = intakeTurretController.calculate(
                        currentTurretAngle,
                        targetTurretAngle
                );
                intakeTurretServo.setPower(-turretPower);
                break;
        }

        bot.telem.addData("Intake Turret State", intakeTurretState.toString());
        bot.telem.addData("Intake Encoder Angle", getEncoderDegrees());
        bot.telem.addData("Current Intake Turret Angle", currentTurretAngle);
        bot.telem.addData("Target Intake Turret Angle", targetTurretAngle);
        bot.telem.addData("Intake Turret Power", intakeTurretServo.getPower());
        bot.telem.addData("Is Intake Limit Switch Pressed", isIntakeTurretLimitSwitchPressed());
        bot.telem.addData("Intake Arm Position", intakeArmServo.getPosition());
        bot.telem.addData("Intake Wrist Position", intakeWristServo.getPosition());
        bot.telem.addData("Intake Claw Position", intakeClawServo.getPosition());
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

    public double getEncoderDegrees() {
        double feedbackVoltage = intakeTurretEncoder.getVoltage();
        return mapVoltageToDegrees(feedbackVoltage);
    }

    private void updateAngleCalculation() {
        // Apply the zero-offset to get a corrected angle relative to the home position.
        double correctedEncoderAngle = (getEncoderDegrees() - encoderZeroOffset + 360) % 360;

        // --- Robust Wrap-Around Detection ---
        // This logic determines if the servo has crossed its 0/360 boundary.
        // It checks if the angle jumped more than halfway around in a single loop.
        double delta = correctedEncoderAngle - previousEncoderAngle;

        if (delta < -180.0) {
            // Wrapped forward (e.g., from 350 to 10)
            servoRevolution++;
        } else if (delta > 180.0) {
            // Wrapped backward (e.g., from 10 to 350)
            servoRevolution--;
        }

        // Update the previous angle for the next loop's calculation.
        previousEncoderAngle = correctedEncoderAngle;

        // --- Final Angle Calculation ---
        // 1. Calculate the servo's total, continuous angle.
        double continuousServoAngle = (servoRevolution * 360.0) + correctedEncoderAngle;

        // 2. Convert the servo's continuous angle to the turret's continuous angle.
        double continuousTurretAngle = continuousServoAngle / INTAKE_TURRET_RATIO;

        // 3. Convert the turret's continuous angle to a final, wrapping 0-360 angle.
        // The double modulo `((... % 360) + 360) % 360` is a robust way
        // to handle negative results if the turret moves past zero.
        currentTurretAngle = (((continuousTurretAngle + 90) % 360.0) + 360.0) % 360.0;
    }

    public void setTurretAngle(double angle) {
        targetTurretAngle = (ServoFunctions.clampAngleDegrees(angle, INTAKE_TURRET_MIN_ANGLE, INTAKE_TURRET_MAX_ANGLE));
    }

    public double getTurretAngle() {
        return currentTurretAngle;
    }

    public void setIntakeTurretPower(double power) {
        intakeTurretServo.setPower(power);
    }

    public boolean isIntakeTurretLimitSwitchPressed() {
        return intakeTurretLimitSwitch.getState();
    }

    public void setIntakeArmAngle(double angle) {
        intakeArmServo.setPosition(ServoFunctions.degreesToServoPosition((ServoFunctions.clampAngleDegrees(angle, INTAKE_ARM_MIN_ANGLE, INTAKE_ARM_MAX_ANGLE)), INTAKE_ARM_SERVO_RANGE * INTAKE_ARM_RATIO));
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
        intakeWristServo.setPosition(ServoFunctions.degreesToServoPosition((ServoFunctions.clampAngleDegrees(angle, INTAKE_WRIST_MIN_ANGLE, INTAKE_WRIST_MAX_ANGLE)), INTAKE_WRIST_SERVO_RANGE * INTAKE_WRIST_RATIO));
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
