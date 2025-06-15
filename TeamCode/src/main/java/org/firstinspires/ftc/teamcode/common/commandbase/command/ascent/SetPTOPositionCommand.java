package org.firstinspires.ftc.teamcode.common.commandbase.command.ascent;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Ascent;

public class SetPTOPositionCommand extends CommandBase {

    private final Ascent ascent;
    private final Ascent.PTOState state;

    public SetPTOPositionCommand(Ascent ascent, Ascent.PTOState state) {
        this.ascent = ascent;
        this.state = state;
        addRequirements(ascent);
    }

    @Override
    public void initialize() {
        switch (state) {
            case IDLE:
                ascent.setPTOPosition(Ascent.PTOState.IDLE);
                break;
            case RELEASE_HOOKS:
                ascent.setPTOPosition(Ascent.PTOState.RELEASE_HOOKS);
                break;
            case ENGAGED:
                ascent.setPTOPosition(Ascent.PTOState.ENGAGED);
                break;
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
