package org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing;

import com.arcrobotics.ftclib.command.CommandBase;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class ApproachLimitSwitch extends CommandBase {
    private final Intake intake;
    private final double power;

    /**
     * Drives the turret at a given power until the limit switch is pressed.
     */
    public ApproachLimitSwitch(Intake intake, double power) {
        this.intake = intake;
        this.power = power;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeTurretPower(power);
    }

    @Override
    public boolean isFinished() {
        return intake.isIntakeTurretLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        // Always stop the motor when the command ends
        intake.setIntakeTurretPower(0.0);
    }
}