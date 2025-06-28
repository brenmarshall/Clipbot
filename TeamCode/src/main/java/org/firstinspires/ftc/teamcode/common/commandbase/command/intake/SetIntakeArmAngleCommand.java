package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class SetIntakeArmAngleCommand extends CommandBase {

    private final Intake intake;
    private final double angle;

    public SetIntakeArmAngleCommand(Intake intake, double angle) {
        this.intake = intake;
        this.angle = angle;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeArmAngle(angle);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
