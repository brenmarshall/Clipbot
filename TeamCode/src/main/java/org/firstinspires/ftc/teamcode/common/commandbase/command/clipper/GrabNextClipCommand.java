package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class GrabNextClipCommand extends SequentialCommandGroup {
    public GrabNextClipCommand(Bot bot) {
        addCommands(
                new SetClipperPositionCommand(bot.getClipper(), Clipper.ClipperState.GRAB),
                new SetClipperDrivePositionCommand(bot.getClipper(), bot.getClipper().magPosition),
                new WaitCommand(250),
                new SetClipperDrivePositionCommand(bot.getClipper(), Configuration.clipDriveClipPosition),
                new SetClipperPositionCommand(bot.getClipper(), Clipper.ClipperState.SET)
        );
    }
}
