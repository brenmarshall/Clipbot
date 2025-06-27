package org.firstinspires.ftc.teamcode.common.commandbase.command.ascent;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Ascent;

public class SetPTOPositionCommand extends CommandBase {

    private final Ascent ascent;
    private final Ascent.PTOState position;

    public SetPTOPositionCommand(Ascent ascent, Ascent.PTOState position) {
        this.ascent = ascent;
        this.position = position;
        addRequirements(ascent);
    }

    @Override
    public void initialize() {
        ascent.setPTOPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
