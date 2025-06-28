package org.firstinspires.ftc.teamcode.common.commandbase.command.intake;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

import java.util.function.DoubleSupplier;

public class ManualTurretCommand extends CommandBase {
    private final Intake intake;
    private final DoubleSupplier leftPower, rightPower;

    public ManualTurretCommand(Intake intake, DoubleSupplier leftPower, DoubleSupplier rightPower) {
        this.intake = intake;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(intake);
    }

    @Override
    public void execute() {
        switch (intake.intakeTurretState) {
            case HOMING:
            case OPERATING:
                break;
            case MANUAL_OVERRIDE:
                intake.setIntakeTurretPower((-rightPower.getAsDouble()) + leftPower.getAsDouble());
                break;
        }
    }
}