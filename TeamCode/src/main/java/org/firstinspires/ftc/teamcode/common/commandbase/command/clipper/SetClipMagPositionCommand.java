package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class SetClipMagPositionCommand extends CommandBase {
    private final Clipper clipper;
    private final Clipper.ClipMagState position;

    public SetClipMagPositionCommand(Clipper clipper, Clipper.ClipMagState position) {
        this.clipper = clipper;
        this.position = position;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setClipMagPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
