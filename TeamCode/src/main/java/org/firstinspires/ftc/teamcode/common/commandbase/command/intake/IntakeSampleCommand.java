package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.common.Bot;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class IntakeSampleCommand extends SequentialCommandGroup {

    public IntakeSampleCommand(Bot bot) {
        addCommands(
                new SetIntakeClawPositionCommand(bot.getIntake(), Intake.IntakeClawState.CLOSED),
                new WaitCommand(100),
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new SetIntakeClawPositionCommand(bot.getIntake(), Intake.IntakeClawState.HOLD),
                                new InstantCommand(() -> bot.getIntake().setIntakeClawState(Intake.IntakeClawState.HOLD))
                        ),
                        new SequentialCommandGroup(
                                new SetIntakeClawPositionCommand(bot.getIntake(), Intake.IntakeClawState.OPEN),
                                new InstantCommand(() -> bot.getIntake().setIntakeClawState(Intake.IntakeClawState.OPEN))
                        ),
                        () -> bot.getIntake().isIntakeClawGrabbing()
                )
        );
    }
}
