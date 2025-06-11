package org.firstinspires.ftc.teamcode.common.commandbase.command.deposit;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.Configuration;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Deposit;

public class SetDepositSlidesPositionCommand extends CommandBase {
    private final Deposit deposit;
    private final double targetPosition;

    public SetDepositSlidesPositionCommand(Deposit deposit, double targetPosition) {
        this.deposit = deposit;
        this.targetPosition = targetPosition;
        addRequirements(deposit);
    }

    @Override
    public void initialize() {
        deposit.setDepositSlidesPosition(targetPosition);
    }

    @Override
    public boolean isFinished() {
        return (Math.abs(deposit.getDepositSlidesPosition() - targetPosition) <= Configuration.depositSlides_tolerance);
    }
}
