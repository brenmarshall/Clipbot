package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class SetClipMagPivotPositionCommand extends CommandBase {
    private final Clipper clipper;
    private final Clipper.ClipMagState position;

    public SetClipMagPivotPositionCommand(Clipper clipper, Clipper.ClipMagState position) {
        this.clipper = clipper;
        this.position = position;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setClipMagPivotPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
