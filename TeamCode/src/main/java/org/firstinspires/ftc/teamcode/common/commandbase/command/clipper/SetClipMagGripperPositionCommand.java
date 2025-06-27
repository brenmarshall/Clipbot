package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class SetClipMagGripperPositionCommand extends CommandBase {

    private final Clipper clipper;
    private final Clipper.ClipMagGripperState position;

    public SetClipMagGripperPositionCommand(Clipper clipper, Clipper.ClipMagGripperState position) {
        this.clipper = clipper;
        this.position = position;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setClipMagGripperPosition(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
