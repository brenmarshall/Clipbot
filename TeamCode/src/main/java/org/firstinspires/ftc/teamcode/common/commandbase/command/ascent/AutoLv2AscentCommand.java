package org.firstinspires.ftc.teamcode.common.commandbase.command.ascent;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.commandbase.command.deposit.SetDepositSlidesPositionCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.command.drive.SetDriveCommand;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Ascent;
import org.firstinspires.ftc.teamcode.common.intothedeep.BotState;

public class AutoLv2AscentCommand extends SequentialCommandGroup {

    public AutoLv2AscentCommand(Bot bot) {
        addCommands(
            new ConditionalCommand(
                    new ParallelCommandGroup(
                            new SetDriveCommand(bot, false),
                            //new SetDepositSlidesPositionCommand(bot.getDeposit(), 0.0),
                            new WaitCommand(250)
                            //new SetDepositSlidesPositionCommand(bot.getDeposit(), 2.0)
                    ),
                    new SequentialCommandGroup(
                            //new SetDepositSlidesPositionCommand(bot.getDeposit(), 0.0),
                            //new SetPTOPositionCommand(bot.getAscent(), Ascent.PTOState.RELEASE_HOOKS),
                            new WaitCommand(250),
                            //new SetDepositSlidesPositionCommand(bot.getDeposit(), Configuration.Lv2LiftHeight),
                            new InstantCommand(() -> bot.setState(BotState.ASCENT))
                    ),
                    () -> bot.getState() == BotState.ASCENT
            )
        );

    }
}
