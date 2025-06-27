package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

public class TestClipCommand extends SequentialCommandGroup {
    public TestClipCommand(Bot bot) {
        addCommands(
            new SetClipperPositionCommand(bot.getClipper(), Clipper.ClipperState.GRAB),
            new SetClipMagPivotPositionCommand(bot.getClipper(), Clipper.ClipMagState.DOWN),
            new HomeClipperDriveCommand(bot.getClipper()),
            new WaitCommand(250),
            new GrabNextClipCommand(bot),
            new WaitCommand(2000),
            new SetClipperPositionCommand(bot.getClipper(), Clipper.ClipperState.CLIP),
            new WaitCommand(500),
            new HomeClipperDriveCommand(bot.getClipper()),
            new SetClipperPositionCommand(bot.getClipper(), Clipper.ClipperState.SET),
            new InstantCommand(() -> bot.getClipper().incrementMagPosition())
        );
    }
}
