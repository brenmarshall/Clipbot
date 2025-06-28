package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class SetIntakeTurretAngleCommand extends CommandBase {
    private final Intake intake;
    private final double targetAngle;

    public SetIntakeTurretAngleCommand(Intake intake, double targetAngle) {
        this.intake = intake;
        this.targetAngle = targetAngle;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setTurretAngle(targetAngle);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(intake.getTurretAngle() - targetAngle) <= Configuration.turret_tolerance);
    }
}
