package org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing;

import com.arcrobotics.ftclib.command.InstantCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class ZeroTurretAtLimit extends InstantCommand {

    /**
     * Instantly performs the encoder zeroing logic for the turret.
     * This should be called immediately after the turret makes contact with the limit switch.
     */
    public ZeroTurretAtLimit(Intake intake) {
        super(
                () -> {

                    intake.encoderZeroOffset = intake.getEncoderDegrees();
                    intake.servoRevolution = 0; // We are at Turret 0°, so we are in Servo Revolution 0.
                    intake.previousEncoderAngle = 90.0; // The corrected angle at home is 0.
                    intake.targetTurretAngle = 90.0;
                    intake.currentTurretAngle = 90.0;
                    intake.intakeTurretState = Intake.IntakeTurretState.POSITION_CONTROL;
                },
                intake
        );
    }
}