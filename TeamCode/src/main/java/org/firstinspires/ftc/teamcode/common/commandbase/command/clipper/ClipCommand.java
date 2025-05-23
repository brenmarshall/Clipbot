package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Config;

public class ClipCommand extends SequentialCommandGroup {
    public ClipCommand(Bot bot) {
        addCommands(
                new InstantCommand(() -> {
                    bot.getClipper().setClipperServoPosition(Config.clippingPosition);
                }),
                new WaitCommand(500),
                new SetClipperDrivePositionCommand(bot.getClipper(), Config.clipperSetPosition),
                new InstantCommand(() -> {
                    bot.getClipper().setClipperServoPosition(Config.pickupPosition);
                })
        );
    }
}
