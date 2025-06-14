package org.firstinspires.ftc.teamcode.common.commandbase.subsystem;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;

public class Ascent extends SubsystemBase {
    private final Bot bot;

    private final DcMotorEx backLeft, backRight, frontLeft, frontRight;
    private final PIDFController ascentController;

    private final Servo leftPTO, rightPTO;

    private double ascentTargetPosition = 0.0;

    public enum PTOState {
        IDLE,
        RELEASE_HOOKS,
        ENGAGED
    }

    public Ascent(Bot bot) {
        this.bot = bot;

        backLeft = bot.hMap.get(DcMotorEx.class, "backLeft");
        backRight = bot.hMap.get(DcMotorEx.class, "backRight");
        frontLeft = bot.hMap.get(DcMotorEx.class, "frontLeft");
        frontRight = bot.hMap.get(DcMotorEx.class, "frontRight");

        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        ascentController = new PIDFController(
                Configuration.ascent_kP,
                Configuration.ascent_kI,
                Configuration.ascent_kD,
                Configuration.ascent_kF
        );

        leftPTO = bot.hMap.get(Servo.class, "leftPTO");
        rightPTO = bot.hMap.get(Servo.class, "rightPTO");

        rightPTO.setDirection(Servo.Direction.REVERSE);
    }

    public void periodic() {
        if (!bot.getEnableDrive()) {
            double targetTicks = ascentTargetPosition * Configuration.depositSlides_ticksPerCM;

            double power = ascentController.calculate(backLeft.getCurrentPosition(), targetTicks);

            backLeft.setPower(power);
            backRight.setPower(power);
            frontLeft.setPower(power);
            frontRight.setPower(power);
        }
    }

    public void setPTOPosition(PTOState state) {
        switch (state) {
            case IDLE:
                leftPTO.setPosition(Configuration.PTOIdlePosition);
                rightPTO.setPosition(Configuration.PTOIdlePosition);
                break;
            case RELEASE_HOOKS:
                leftPTO.setPosition(Configuration.PTOReleaseHooksPosition);
                rightPTO.setPosition(Configuration.PTOReleaseHooksPosition);
                break;
            case ENGAGED:
                leftPTO.setPosition(Configuration.PTOEngagedPosition);
                rightPTO.setPosition(Configuration.PTOEngagedPosition);
                break;
        }
    }
}
