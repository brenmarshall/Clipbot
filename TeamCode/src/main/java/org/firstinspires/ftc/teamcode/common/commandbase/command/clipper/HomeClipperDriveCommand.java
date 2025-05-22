package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class HomeClipperDriveCommand extends CommandBase {
    private final Clipper clipper;

    public HomeClipperDriveCommand(Clipper clipper) {
        this.clipper = clipper;
        addRequirements(clipper);
    }

    @Override
    public void initialize() {
        clipper.setIsHoming(true);
        clipper.setClipperDrivePower(0.25);
    }

    @Override
    public boolean isFinished() {
        return clipper.isClipperLimitSwitchPressed();
    }

    @Override
    public void end(boolean interrupted) {
        clipper.setClipperDrivePower(0);
        clipper.resetClipperEncoder();
        clipper.setClipperDrivePosition(0.0);
        clipper.setIsHoming(false);
    }
}
