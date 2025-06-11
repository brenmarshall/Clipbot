package org.firstinspires.ftc.teamcode.common.commandbase.command.clipper;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;

import java.util.function.DoubleSupplier;

public class ManualClipperDriveCommand extends CommandBase {
    private final Clipper clipper;
    private final DoubleSupplier leftPower, rightPower;

    public ManualClipperDriveCommand(Clipper clipper, DoubleSupplier leftPower, DoubleSupplier rightPower) {
        this.clipper = clipper;
        this.leftPower = leftPower;
        this.rightPower = rightPower;
        addRequirements(clipper);
    }

    @Override
    public void execute() {
        clipper.setClipperDrivePower((-leftPower.getAsDouble()) + rightPower.getAsDouble());
    }
}
