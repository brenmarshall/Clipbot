package org.firstinspires.ftc.teamcode.common.commandbase.command.intake.homing;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class BackoffFromLimitSwitch extends CommandBase {
    private final Intake intake;
    private final double power;
    private final long durationMs;
    private ElapsedTime timer;

    /**
     * Drives the turret at a negative power for a specific duration in milliseconds.
     */
    public BackoffFromLimitSwitch(Intake intake, double power, long durationMs) {
        this.intake = intake;
        // Ensure power is negative for backing off
        this.power = -Math.abs(power);
        this.durationMs = durationMs;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer = new ElapsedTime();
        intake.setIntakeTurretPower(power);
    }

    @Override
    public boolean isFinished() {
        return timer.milliseconds() >= durationMs;
    }

    @Override
    public void end(boolean interrupted) {
        intake.setIntakeTurretPower(0.0);
    }
}