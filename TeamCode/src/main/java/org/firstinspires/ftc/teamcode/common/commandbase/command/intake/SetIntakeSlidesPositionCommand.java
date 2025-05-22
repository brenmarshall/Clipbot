package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class SetIntakeSlidesPositionCommand extends CommandBase {
    private final Intake intake;
    private final double targetPosition;

    public SetIntakeSlidesPositionCommand(Intake intake, double targetPosition) {
        this.intake = intake;
        this.targetPosition = targetPosition;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeSlidesPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(intake.getIntakeSlidesPosition() - targetPosition) <= Config.intakeSlides_tolerance);
    }
}
