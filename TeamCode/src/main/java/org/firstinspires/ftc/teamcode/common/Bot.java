package org.firstinspires.ftc.teamcode.common;

import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;

public class Bot extends Robot {
    public final Telemetry telem;
    public final HardwareMap hMap;
    public final Gamepad gamepad;

    private final Intake intake;
    private final Clipper clipper;
    //private final Deposit deposit;

    private boolean enableDrive = true;

    public Bot(Telemetry telem, HardwareMap hMap, Gamepad gamepad) {
        this.telem = telem;
        this.hMap = hMap;
        this.gamepad = gamepad;

        intake = new Intake(this);
        clipper = new Clipper(this);
        //deposit = new Deposit(this);
    }

    public Intake getIntake() {
        return intake;
    }

    public Clipper getClipper() {
        return clipper;
    }

    //public Deposit getDeposit() {
        //return deposit;
    //}

    public void setEnableDrive(boolean enableDrive) { this.enableDrive = enableDrive; }

    public boolean getEnableDrive() { return enableDrive; }
}
