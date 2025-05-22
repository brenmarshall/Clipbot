package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class IntakeToSamplePositionCommand extends CommandBase {
    private final Intake intake;
    private final Pose2d target;

    public IntakeToSamplePositionCommand(Intake intake, Pose2d target) {
        this.intake = intake;
        this.target = target;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
    }
}
