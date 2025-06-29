package org.firstinspires.ftc.teamcode.common;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Clipper;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Deposit;
import org.firstinspires.ftc.teamcode.common.commandbase.subsystem.Intake;
import org.firstinspires.ftc.teamcode.common.intothedeep.BotState;

public class Bot extends Robot {
    public static FtcDashboard dashboard;
    public final Telemetry telem;
    public final HardwareMap hMap;
    public final Gamepad gamepad;

    private final Intake intake;
    private final Clipper clipper;
    private final Deposit deposit;

    public static BotState state = BotState.SCORING;

    private boolean enableDrive = true;

    public Bot(Telemetry telem, HardwareMap hMap, Gamepad gamepad) {
        this.telem = telem;
        this.hMap = hMap;
        this.gamepad = gamepad;

        dashboard = FtcDashboard.getInstance();

        intake = new Intake(this);
        clipper = new Clipper(this);
        deposit = new Deposit(this);
    }

    public Intake getIntake() {
        return intake;
    }

    public Clipper getClipper() {
        return clipper;
    }

    public Deposit getDeposit() {
        return deposit;
    }

    public void setState(BotState state) {
        Bot.state = state;
    }

    public BotState getState() {
        return state;
    }

    public void setEnableDrive(boolean enableDrive) { this.enableDrive = enableDrive; }

    public boolean getEnableDrive() { return enableDrive; }
}
