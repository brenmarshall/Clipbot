package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class SetClipperPositionCommand extends CommandBase {
    private final Clipper clipper;
    private final Clipper.ClipperState position;

    public SetClipperPositionCommand(Clipper clipper, Clipper.ClipperState position) {
        this.clipper = clipper;
        this.position = position;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setClipperPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
