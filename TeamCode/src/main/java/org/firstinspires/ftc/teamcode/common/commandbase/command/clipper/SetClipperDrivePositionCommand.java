package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Config;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class SetClipperDrivePositionCommand extends CommandBase {
    private final Clipper clipper;
    private final double position;

    public SetClipperDrivePositionCommand(Clipper clipper, double position) {
        this.clipper = clipper;
        this.position = position;
        addRequirements(clipper);
    }

    public SetClipperDrivePositionCommand(Clipper clipper, int position) {
        this.clipper = clipper;
        this.position = (position * clipper.clipWidth) + Config.clipMagOffset;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setClipperDrivePosition(position);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(clipper.getClipperDrivePosition() - position) <= Config.clipperDrive_tolerance);
    }
}
