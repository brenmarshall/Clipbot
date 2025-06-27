
package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class SetIntakeClawPositionCommand extends CommandBase {

    private Intake intake;
    private Intake.IntakeClawState state;

    public SetIntakeClawPositionCommand(Intake intake, Intake.IntakeClawState state) {
        this.intake = intake;
        this.state = state;
        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIntakeClawPosition(state);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
