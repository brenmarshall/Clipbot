package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

public class GrabNextClipCommand extends SequentialCommandGroup {
    public GrabNextClipCommand(Bot bot) {
        addCommands(
                new InstantCommand(() -> {
                    bot.getClipper().setClipperServoPosition(Config.pickupPosition);
                }),
                new SetClipperDrivePositionCommand(bot.getClipper(), bot.getClipper().magPosition),
                new WaitCommand(500),
                new SetClipperDrivePositionCommand(bot.getClipper(), Config.clipperSetPosition),
                new InstantCommand(() -> {
                    bot.getClipper().setClipperServoPosition(Config.clippingPosition);
                }),
                new WaitCommand(500),
                new SetClipperDrivePositionCommand(bot.getClipper(), Config.clipperClipPosition)
        );
    }
}
